(ns slicer-test
  (:require [slicer :refer [gcode-cmd->str
                            rapid-move
                            home
                            stop]]
            [clojure.test :refer [deftest are]]))

(deftest gcode-cmd->str-test
  (are [cmd s] (= s (gcode-cmd->str cmd))
    (rapid-move {:X 20 :C 10 :Z 40}) "G0 X20 Z40 C10"
    (rapid-move {:X 20 :C 10 :Z 40} :feedrate 20) "G0 X20 Z40 C10 F20"
    (rapid-move {:Z 12 :C 10} :feedrate 10) "G0 Z12 C10 F10"
    (home) "G28"
    (stop) "M0"))
