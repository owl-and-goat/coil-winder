(ns slicer
  (:refer-clojure :exclude [run!])
  (:require [clojure.string :as str]
            [clojure.math :as math]
            [babashka.fs :as fs]
            [clojure.java.io :as io]
            [clojure.java.shell :refer [sh]]))

(def coords [:X :Z :C :F])
(def feedrate-coord :F)

(defn rapid-move [coords & {:keys [feedrate]}]
  [:G0 (assoc coords feedrate-coord feedrate)])
(defn linear-move [coords & {:keys [feedrate]}]
  [:G0 (assoc coords feedrate-coord feedrate)])
(defn home [& {:keys [feedrate]}]
  (into [:G28]
        (when feedrate [{feedrate-coord feedrate}])))
(defn stop [] [:M0])
(defn emergency-stop [] [:M112])
(defn enable-all-steppers [] [:M17])
(defn disable-all-steppers [] [:M18])
(defn get-current-position [] [:M114])
(defn pause [] [:M226])

(defn gcode-atom->str [elt]
  (cond
    (keyword? elt) (name elt)
    (string? elt) (name elt)

    (and (vector? elt)
         (keyword? (first elt))
         (number? (second elt)))
    (str (name (first elt))
         (if (ratio? (second elt))
           (-> elt second float str)
           (-> elt second str)))

    (map? elt)
    (->> coords
         (keep (fn [coord]
                 (when (contains? elt coord)
                   [coord (elt coord)])))
         (map gcode-atom->str)
         (str/join " "))))

(defn gcode-cmd->str [gcode-cmd]
  (->> gcode-cmd
       (map gcode-atom->str)
       (str/join " ")
       str/trim))

(defn gcode->str [gcode]
  (str (->> gcode (map gcode-cmd->str) (str/join "\n")) "\n"))

(def preamble [(stop)
               (enable-all-steppers)
               (home)])

(defn gcode-program [commands]
  (into
   []
   (concat preamble commands [(disable-all-steppers)])))

(comment
  (println
   (gcode->str
    (gcode-program [(rapid-move {:X 20 :C 10 :Z 40})]))))

;;;
;;; Running gcode programs
;;;

(def project-dir
  (-> "."
      io/file
      fs/canonicalize
      fs/parent))

(def programs-dir
  (fs/path project-dir "programs"))

(def coil-winder-client-path
  (fs/path project-dir "client/target/release/client"))

(defn coil-winder-client! [& args]
  (apply sh (str coil-winder-client-path) args))

(defn oneshot! [gcode]
  (let [gcode-str (gcode-cmd->str gcode)]
    (println "RUNNING: " gcode-str)
    (coil-winder-client! "oneshot" "-c" gcode-str)))

(defn run! [gcode]
  (coil-winder-client! "run" "-" :in (gcode->str gcode)))

;;;
;;; Coil programs
;;;

(defn scramble-wind [{:keys [turns pause-before-execute? feedrate]
                      bobbin-position :bobbin/position
                      bobbin-width :bobbin/width
                      wire-width :wire/width
                      :or {feedrate 20
                           pause-before-execute? true}}]
  (let [;; step to beginning of bobbin
        step-to-beginning (rapid-move
                           {:Z bobbin-position :X 0}
                           :feedrate feedrate)
        turns-per-layer (/ bobbin-width wire-width)
        mk-turn-positions
        (fn []
          (->> (range 0 turns-per-layer)
               (map (fn [pos] (+ (* pos wire-width)
                                 bobbin-position)))
               shuffle
               (map (fn [z] {:Z z}))))
        mk-layer
        (fn [layer-idx c-start]
          (map (fn [coord c]
                 (assoc coord
                        :C (+ c-start c)
                        :X (* wire-width layer-idx)))
               (mk-turn-positions)
               (range)))
        num-layers (math/ceil (/ turns turns-per-layer))]
    (concat
     [step-to-beginning]
     (when pause-before-execute? [(pause)])
     (->> num-layers
          range
          (map-indexed mk-layer)
          (mapcat identity)
          (take turns)
          (map rapid-move)))))

(defn linear-wind [{bobbin-position :bobbin/position
                    bobbin-width :bobbin/width
                    wire-width :wire/width
                    backoff-x :backoff/x
                    backoff-z :backoff/z}]
  (let [base-feedrate 20
        turns (math/floor (/ bobbin-width wire-width))
        backoff-turns (math/ceil (/ backoff-z wire-width))

        wind (fn [turns c-start z-start]
               (rapid-move
                {:C (+ c-start turns)
                 :Z (+ z-start (* turns wire-width))}))

        step-to-beginning (rapid-move
                           {:Z bobbin-position :X 0}
                           :feedrate base-feedrate)
        back-off (rapid-move
                  {:Z bobbin-position :X backoff-x})
        step-to-end (rapid-move
                     {:Z (+ bobbin-position bobbin-width)})]
    [step-to-beginning
     (wind backoff-turns 0 bobbin-position)
     back-off
     (wind (- turns backoff-turns) backoff-turns bobbin-position)
     step-to-end]))

(comment
  (run! [(enable-all-steppers)
         (home :feedrate 20)
         (rapid-move {:X 50 :Z 50} :feedrate 30)
         (disable-all-steppers)])

  (def program
    (scramble-wind {:turns 5000
                    :bobbin/position 43.5
                    :bobbin/width 8.25
                    :wire/width 0.06335}))

  (->> (scramble-wind {:turns 5000
                       :bobbin/position 42.35
                       :bobbin/width 8.25
                       :wire/width 0.06335
                       :feedrate 5})
       gcode-program
       gcode->str
       (spit (str (fs/path programs-dir "pickup-1.gcode"))))

  (oneshot! (first program))

  (run! (concat preamble [(first program)]))
  (run! (rest program))
  (run! (concat preamble program))
  (oneshot! (stop))
  (oneshot! (emergency-stop))
  (oneshot! (get-current-position))
  (oneshot! (pause)))
