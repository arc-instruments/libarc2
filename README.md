# ArC TWO™ Low-level library

This is the low-level interface library for ArC TWO. It manages the tool's
command buffer, memory and instruction serialisation. It also provides some
basic testing-related functionality such as individual pulsing and reading as
well as a generic ramp generator. It is meant to be used as a bulding block
towards developing more elaborate testing setups.

## Using

The recommended way to get started with `libarc2` is through its [Python
bindings](https://github.com/arc-instruments/pyarc2). If you do, however,
want to hack on `libarc2` itself or build Rust-based testing programs then
you can do so by adding the following to you `Cargo.toml`.

```toml
[dependencies]
libarc2 = "0.4"
```

The library is still under development and more features are added constantly.
An effort is made to maintain a consistent API but there is no API stability
guarantee until we hit 1.0.

## Requirements

To actually talk to an ArC TWO board you will need the beastlink FPGA library.
Detailed instructions are available on the
[beastlink-rs](https://github.com/arc-instruments/beastlink-rs#prerequisites)
repository. The Python wheels of the `libarc2` bindings include all necessary
libraries but if you are using `libarc2` itself you need to make them available
for the Rust compiler to link against. For Windows a Python script is included
to pull the necessary DLLs into the working directory. For Linux follow the
instructions on the beastlink-rs repository to build appropriate packages for
your distribution (for Debian, RedHat, Archlinux and their derivatives).
