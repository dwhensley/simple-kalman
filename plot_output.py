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
    else:
        raise ValueError("Did not understand selector, must be one of {{0, 1, 2}}")

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

    plt.plot(inputs, 'ob', alpha=0.25)
    plt.plot(outputs, 'r')
    plt.plot(truth, '--g')

    plt.title(f"{lang} Scalar Kalman Filter Example")
    plt.legend(["input", "kalman filter output", "truth"])

    plt.show()


if __name__ == "__main__":
    main()