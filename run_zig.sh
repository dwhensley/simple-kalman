#!/bin/bash

mkdir -p outputs
./targetz/kalman > outputs/out_zig.csv
python plot_output.py 1
