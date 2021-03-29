#!/bin/bash

cargo build --release
mkdir -p targetz
zig build-exe src/main.zig -femit-bin=targetz/kalman -O ReleaseSafe
mkdir -p targetc
gcc -Wall -Werror -fpic -O3 src/main.c -o targetc/kalman -lm
