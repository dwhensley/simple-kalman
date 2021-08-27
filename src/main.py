""" Python Scalar Kalman Filter

Implementation of simple scalar Kalman filter with no control (no `B * u` term).
"""

from dataclasses import dataclass

import numpy as np


NUM_ITER = 100


@dataclass
class ScalarKalman:
    A: float
    H: float
    Q: float
    R: float
    x: float = 0.0
    P: float = 0.0

    def predict(self):
        self.x *= self.A
        self.P = self.A * self.P * self.A + self.Q

    def update(self, z: float):
        y = z - self.H * self.x
        S = self.H * self.P * self.H + self.R
        if np.abs(S) < 1e-8:
            raise ValueError(
                "failed to invert scalar innovation (measurement pre-fit residual `S`)"
            )
        S_inv = 1.0 / S
        K = self.P * self.H * S_inv
        self.x += K * y
        self.P *= 1.0 - K * self.H

    def advance(self, z: float) -> float:
        self.predict()
        self.update(z)
        return self.x


def main():
    process_noise_std = 0.075
    R = process_noise_std ** 2

    delt = 0.1
    t = np.linspace(0.0, NUM_ITER * delt, NUM_ITER)
    f = 0.1
    w = 2.0 * np.pi * f
    phase = 0.0
    magnitude = 0.2
    dc_offset = 0.2294

    x_truth = magnitude * np.sin(w * t - phase) + dc_offset
    noise = np.array(
        [np.random.normal(0.0, process_noise_std) for _ in range(len(x_truth))]
    )
    z_obs = x_truth + noise

    A = 1.0
    H = 1.0
    Q = 1e-4

    x0 = x_truth[0]
    kalman_filter = ScalarKalman(A, H, Q, R, x0)

    out = np.zeros_like(z_obs)
    for idx, z in enumerate(z_obs):
        out[idx] = kalman_filter.advance(z)

    for x, z, o in zip(x_truth, z_obs, out):
        print(f"{x},{z},{o}")


if __name__ == "__main__":
    main()
