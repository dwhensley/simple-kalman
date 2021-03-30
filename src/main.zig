//! Zig implementation of simple scalar Kalman filter with no control (no `B * u` term).

const std = @import("std");

const Float = f32;

const KalmanError = error {
    FailedScalarInverse,
};

const ScalarKalman = struct {
    x: Float,
    P: Float,
    A: Float,
    H: Float,
    Q: Float,
    R: Float,

    fn init(
        A: Float,
        H: Float,
        Q: Float,
        R: Float,
        x0: ?Float,
        P0: ?Float
    ) ScalarKalman {
        const x = x0 orelse 0.0;
        const P = P0 orelse 0.0;
        return ScalarKalman {
            .x = x,
            .P = P,
            .A = A,
            .H = H,
            .Q = Q,
            .R = R,
        };
    }

    fn predict(self: *ScalarKalman) void {
        self.x *= self.A;
        self.P = self.A * self.P * self.A + self.Q;
    }

    fn update(self: *ScalarKalman, z: Float) KalmanError!void {
        const y = z - self.H * self.x;
        const S = self.H * self.P * self.H + self.R;
        if (std.math.absFloat(S) < 1e-8) {
            return KalmanError.FailedScalarInverse;
        }
        const S_inv = 1.0 / S;
        const K = self.P * self.H * S_inv;
        self.x += K * y;
        self.P *= 1.0 - K * self.H;
    }

    fn advance(self: *ScalarKalman, z: Float) KalmanError!Float {
        self.predict();
        try self.update(z);
        return self.x;
    }
};

pub fn main() anyerror!void {
    const process_noise_max: Float = 0.2;
    const num_iter: usize = 100;
    const PI: Float = 3.14159265359;
    const f: Float = 0.1;
    const w: Float = 2.0 * PI * f;
    const phase: Float = 0.0;
    const magnitude: Float = 0.2;
    const dc_offset: Float = 0.2294;

    var t: [num_iter]Float = undefined;
    const delt: Float = 0.1;
    var currt: Float = 0.0;
    for (t) |*item| {
        item.* = currt;
        currt += delt;
    }

    var x_truth: [num_iter]Float = undefined;
    for (x_truth) |*x, idx| {
        x.* = magnitude * std.math.sin(w * t[idx] - phase) + dc_offset;
    }

    var rng = std.rand.Xoroshiro128.init(2374554);
    var z_obs: [num_iter]Float = undefined;
    for (z_obs) |*z, idx| {
        const noise = process_noise_max * (rng.random.float(Float) - 0.5);
        z.* = x_truth[idx] + noise;
    }

    const A: Float = 1.0;
    const H: Float = 1.0;
    const Q: Float = 1e-4;
    const R: Float = 0.05 * 0.05;
    const x0 = z_obs[0];

    var kalman_filter = ScalarKalman.init(A, H, Q, R, x0, null);

    var output: [num_iter]Float = undefined;
    for (output) |*o, idx| {
        o.* = try kalman_filter.advance(z_obs[idx]);
    }

    const stdout = std.io.getStdOut().writer();
    var idx: usize = 0;
    while (idx < num_iter) : (idx += 1) {
        try stdout.print("{},", .{x_truth[idx]});
        try stdout.print("{},", .{z_obs[idx]});
        try stdout.print("{}\n", .{output[idx]});
    }
}
