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

```
[dependencies]
libarc2 = { git = "https://github.com/arc-instruments/libarc2" }
```

As the library stabilises regular Rust crates will be made available.

## Requirements

To actually talk to an ArC TWO board you will need the beastlink FPGA library.
Detailed instructions are available on the
[beastlink-rs](https://github.com/arc-instruments/beastlink-rs#prerequisites)
repository. For Windows a Python script is included to pull the necessary DLLs
into the working directory if you intend to build `libarc2` yourself.
