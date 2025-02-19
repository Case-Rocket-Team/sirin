use std::path::{Path, PathBuf};
use walkdir::WalkDir;

pub fn main() {
    cc::Build::new()
        .compiler("arm-none-eabi-gcc")
        .includes(recursive_dirs("cmsis/CMSIS/Core/Include"))
        .includes(recursive_dirs("cmsis-dsp/PrivateInclude"))
        .includes(recursive_dirs("cmsis-dsp/Include"))
        .includes(recursive_dirs("cmsis-dsp/Source"))
        .file("src/sirin-c.c")
        .file("cmsis-dsp/Source/CommonTables/CommonTables.c")
        .file("cmsis-dsp/Source/FastMathFunctions/FastMathFunctions.c")
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