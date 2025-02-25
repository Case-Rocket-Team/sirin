This crate contains the C component of Sirin; it has both Rust and C parts, and the Rust part is essentially only the FFI for the C part. The C part is built as a part of `cargo build` as defined in `build.rs`, where `cc::Build` describes the `arm-none-eabi-gcc` command to build the C part.

# Setup
1. Install the ARM compiler `arm-none-eabi-gcc` (neat install script for Linux: https://askubuntu.com/a/1371525)
2. Verify your ARM compiler installation with `arm-none-eabi-gcc --version`

# CMSIS-DSP
From the CMSIS-DSP readme:
> In Source subfolders, you may either build all of the source file with a datatype suffix (like _f32.c), or just compile the files without a datatype suffix. For instance for BasicMathFunctions, you can build all the C files except BasicMathFunctions.c and BasicMathFunctionsF16.c, or you can just build those two files (they are including all of the other C files of the folder).

Includes and C files need to be specified in `build.rs` (see, for example, for `FastMathFunctions.c`).