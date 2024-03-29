import sys
import os

import csv
import numpy as np
import matplotlib.pyplot as plt


def main():

    lang_selector = int(sys.argv[1])
    if lang_selector == 0:
        filename = "out_rust.csv"
        lang = "Rust"
    elif lang_selector == 1:
        filename = "out_zig.csv"
        lang = "Zig"
    elif lang_selector == 2:
        filename = "out_c.csv"
        lang = "C"
    elif lang_selector == 3:
        filename = "out_py.csv"
        lang = "Python"
    else:
        raise ValueError("Did not understand selector, must be one of {{0, 1, 2, 3}}")

    with open(os.path.join("outputs", filename)) as f:
        rdr = csv.reader(f)
        data = []
        for v in rdr:
            data.append(v)

    truth = []
    inputs = []
    outputs = []

    for row in data:
        truth.append(float(row[0]))
        inputs.append(float(row[1]))
        outputs.append(float(row[2]))

    truth = np.array(truth)
    inputs = np.array(inputs)
    outputs = np.array(outputs)
    time = np.linspace(0.0, 10.0, len(inputs))

    plt.plot(time, inputs, 'ob', alpha=0.25)
    plt.plot(time, outputs, 'r')
    plt.plot(time, truth, '--g')

    plt.title(f"{lang} Scalar Kalman Filter Example")
    plt.xlabel("Time [s]")
    plt.legend(["input", "kalman filter output", "truth"])

    plt.show()


if __name__ == "__main__":
    main()