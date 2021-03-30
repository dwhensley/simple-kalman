//! Rust implementation of simple scalar Kalman filter with no control (no `B * u` term).
#![allow(non_snake_case)]

use anyhow::Result;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};
use thiserror::Error;

type Float = f64;
const PI: Float = std::f64::consts::PI;
const NUM_ITER: usize = 100;

#[derive(Error, Debug)]
enum KalmanError {
    #[error("failed to invert scalar {scalar_name} in operation")]
    FailedScalarInverse { scalar_name: &'static str },
}
type KalmanResult<T> = std::result::Result<T, KalmanError>;

struct ScalarKalman {
    x: Float,
    P: Float,
    A: Float,
    H: Float,
    Q: Float,
    R: Float,
}

impl ScalarKalman {
    fn new(A: Float, H: Float, Q: Float, R: Float, x0: Option<Float>, P0: Option<Float>) -> Self {
        let x = if let Some(x0) = x0 { x0 } else { 0.0 };
        let P = if let Some(P0) = P0 { P0 } else { 0.0 };
        Self { x, P, A, H, Q, R }
    }

    fn predict(&mut self) {
        self.x *= self.A;
        self.P = self.A * self.P * self.A + self.Q;
    }

    fn update(&mut self, z: Float) -> KalmanResult<()> {
        let y = z - self.H * self.x;
        let S = self.H * self.P * self.H + self.R;
        if S.abs() < 1e-8 {
            return Err(KalmanError::FailedScalarInverse {
                scalar_name: "Innovation (measurement pre-fit residual `S`)",
            });
        }
        let S_inv = 1. / S;
        let K = self.P * self.H * S_inv;
        self.x += K * y;
        self.P *= 1.0 - K * self.H;
        Ok(())
    }

    fn advance(&mut self, z: Float) -> KalmanResult<Float> {
        self.predict();
        self.update(z)?;
        Ok(self.x)
    }
}

fn main() -> Result<()> {
    let process_noise_std = 0.075;
    let R = process_noise_std * process_noise_std;

    let delt = 0.1;
    let mut t: [Float; NUM_ITER] = [0.0; NUM_ITER];
    t.iter_mut()
        .zip(0..100)
        .for_each(|(t, n)| *t = delt * n as Float);
    let f = 0.1;
    let w = 2.0 * PI * f;
    let phase = 0.0;
    let magnitude = 0.2;
    let dc_offset = 0.2294;

    let mut x_truth: [Float; NUM_ITER] = [0.0; NUM_ITER];
    for (x, &t) in x_truth.iter_mut().zip(t.iter()) {
        *x = magnitude * (w * t - phase).sin() + dc_offset;
    }

    let normal_dist = Normal::new(0.0, process_noise_std)?;
    let mut z_obs: [Float; NUM_ITER] = [0.0; NUM_ITER];
    for (z, &x) in z_obs.iter_mut().zip(x_truth.iter()) {
        *z = x + normal_dist.sample(&mut thread_rng());
    }

    let A = 1.0;
    let H = 1.0;
    let Q = 1e-4;

    let x0 = x_truth[0];
    let mut kalman_filter = ScalarKalman::new(A, H, Q, R, Some(x0), None);

    let mut output: [Float; NUM_ITER] = [0.0; NUM_ITER];
    for (out, &z) in output.iter_mut().zip(z_obs.iter()) {
        *out = kalman_filter.advance(z)?;
    }

    for (&x, (&z, &out)) in x_truth.iter().zip(z_obs.iter().zip(output.iter())) {
        print!("{},", x);
        print!("{},", z);
        println!("{}", out);
    }
    Ok(())
}
