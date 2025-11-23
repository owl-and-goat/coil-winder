(ns slicer
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
(defn home [] [:G28])
(defn stop [] [:M0])
(defn enable-all-steppers [] [:M17])
(defn disable-all-steppers [] [:M18])
(defn get-current-position [] [:M114])

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
    (gcode-program [(rapid-move {:X 20 :C 10 :Z 40})])))
  )

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
  (apply sh coil-winder-client-path args))

(defn oneshot! [gcode]
  (let [gcode-str (gcode-cmd->str gcode)]
    (println "RUNNING: " gcode-str)
    (coil-winder-client! "oneshot" "-c" gcode-str)))

(defn run! [gcode]
  (coil-winder-client! "run" "-" :in (gcode->str gcode)))

;;;
;;; Coil programs
;;;

(defn scramble-wind [{:keys [turns]
                      bobbin-position :bobbin/position
                      bobbin-width :bobbin/width
                      wire-width :wire/width}]
  (let [base-feedrate 20
        ;; step to beginning of bobbin
        step-to-beginning (rapid-move
                           {:Z bobbin-position :X 0}
                           :feedrate base-feedrate)
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
     (->> num-layers
          range
          (map-indexed mk-layer)
          (mapcat identity)
          (take turns)
          (map rapid-move)))))

(comment
  (def program
    (scramble-wind {:turns 100
                    :bobbin/position 43
                    :bobbin/width 13.4
                    :wire/width 0.13
                    }))

  (->> program
       gcode-program
       gcode->str
       (spit (str (fs/path programs-dir "scramble-wind.gcode"))))

  (oneshot! (first program))

  (run! (concat preamble [(first program)]))
  (run! (rest program))
  (oneshot! (stop))
  (oneshot! (get-current-position))

  (oneshot! (enable-all-steppers))
  (oneshot! (disable-all-steppers))
  (oneshot! (home))
  (oneshot! (disable-all-steppers))
  (run! [(enable-all-steppers) (home)])
  )
