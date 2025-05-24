use std::path::{Path, PathBuf};
use walkdir::WalkDir;

pub fn main() {
    println!("cargo::rerun-if-changed=src/*");

    cc::Build::new()
        .compiler("arm-none-eabi-gcc")
        .includes(recursive_dirs("cmsis/CMSIS/Core/Include"))
        .includes(recursive_dirs("cmsis-dsp/PrivateInclude"))
        .includes(recursive_dirs("cmsis-dsp/Include"))
        .includes(recursive_dirs("cmsis-dsp/Source"))
        .file("src/sirin-c.c")
        .file("cmsis-dsp/Source/CommonTables/CommonTables.c")
        .file("cmsis-dsp/Source/FastMathFunctions/FastMathFunctions.c")
        .file("cmsis-dsp/Source/BasicMathFunctions/BasicMathFunctions.c")
        .file("cmsis-dsp/Source/QuaternionMathFunctions/QuaternionMathFunctions.c")
        .file("cmsis-dsp/Source/MatrixFunctions/MatrixFunctions.c")
        .target("thumbv7em-none-eabihf")
        .compile("sirin-c");
}

fn recursive_dirs<P: AsRef<Path>>(dir: P) -> Vec<PathBuf> {
    let mut vec: Vec<PathBuf> = vec![];

    for entry in WalkDir::new(dir) {
        let Ok(entry) = entry else {
            continue;
        };

        if entry.file_type().is_dir() {
            vec.push(entry.into_path())
        }
    }

    return vec;
}

#[allow(dead_code)]
fn recursive_files<P: AsRef<Path>>(dir: P) -> Vec<PathBuf> {
    let mut vec: Vec<PathBuf> = vec![];

    for entry in WalkDir::new(dir) {
        let Ok(entry) = entry else {
            continue;
        };

        let filename = entry.file_name().to_str().unwrap();

        if entry.file_type().is_file() && (filename.ends_with(".h") || filename.ends_with(".c")) {
            vec.push(entry.into_path())
        }
    }

    return vec;
}