# Simple Kalman

This repo demos a simple scalar Kalman filter (linear quadratic estimator) in three languages:

1. [Rust](https://www.rust-lang.org/)
2. [Zig](https://ziglang.org/)
3. Venerable [C](https://en.wikipedia.org/wiki/C_(programming_language))

The filter is implemented in a single `main` file for each language using a very similar style. They are not exactly the same. For example, the Rust program adds noise by drawing from a normal distribution while the Zig and C programs draw additive noise points evenly from a linear range.

The programs are simple so comparisons across the languages is limited. But this gives a small flavor and working example code across all three for those interested in taking a look at Rust and/or Zig.

## Running the Demos

To run the demos, you will need compilers for all three languages (see below).

With the compilers installed, you can:

1. Build everything by executing `build_all.sh` in your shell (*e.g.*, `bash build_all.sh` or `zsh build_all.sh`).
2. Run the Rust demo via `run_rust.sh`.
3. Run the Zig demo via `run_zig.sh`.
4. Run the C demo via `run_c.sh`.

## The Source code

The source code is in `src` where there is a single `main` file for each of the 3 languages.

### Compiling Rust

It is recommended to use `rustup`. Follow the [instructions here](https://www.rust-lang.org/tools/install).

### Compiling Zig

Following the [instructions here](https://ziglang.org/learn/getting-started/#installing-zig) for your preferred method of installing the `zig` compiler on your target platform.

### Compiling C

The demos assume [GCC](https://gcc.gnu.org/) is available on your system. On Macs, `gcc` may be an alias for `clang` which should work just fine.

## The Kalman Filter

All demos run the same [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) model. The model is linear, scalar, and does not include control inputs (*e.g.*, a `B * u` term). The same terse variable names are used in all language demos and reflect the general mathematical components of the Kalman Filter:

- `x`: State variable (in this case, just a scalar).
- `P`: *A posteriori* estimate of variance.
- `A`: The state transition/physics forward model.
- `H`: The observation model.
- `Q`: Variance for the process noise.
- `R`: Variance for the observation noise.

The Kalman filter is implemented with a stateful `struct` in each language and a two-step process to evaluate the output of the filter given a new observation. This common split -- first `predict` and then `update` -- is reflected in two separate functions.
