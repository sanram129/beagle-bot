#!/bin/bash

motor_pins=("P8_45" "P8_46" "P8_43" "P8_44" "P8_41" "P8_42" \
            "P8_39" "P8_40" "P8_27" "P8_29" "P8_28" "P8_30")

for pin in ${motor_pins[@]}; do
    config-pin $pin pruout
done