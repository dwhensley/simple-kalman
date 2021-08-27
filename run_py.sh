#!/bin/bash

mkdir -p outputs
python src/main.py > outputs/out_py.csv
python plot_output.py 3
