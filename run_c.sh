#!/bin/bash

mkdir -p outputs
./targetc/kalman > outputs/out_c.csv
python plot_output.py 2
