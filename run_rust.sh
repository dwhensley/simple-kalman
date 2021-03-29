#!/bin/bash

mkdir -p outputs
cargo run --release > outputs/out_rust.csv
python plot_output.py 0
