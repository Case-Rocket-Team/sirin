fn main(){
    println!("cargo::rerun-if-changed=c");

    cc::Build::new()
        .file("c/extended_kalman_filter.c")
        .file("c/base/base.c")
        .flag("-std=c2x")
        .flag("-fno-math-errno")
        .compile("extended_kalman");
}