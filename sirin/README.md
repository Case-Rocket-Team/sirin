# About
This is code for Sirin, a flight computer. It **may not build** when used from `crates.io`, please use the GitHub repo to build it instead.

# Architecture
Sirin's code is organized as a Cargo workspace, where the "main" crate is the `sirin` crate. `sirin` then depends on all of the other crates (e.g. `bmp3`, `lsm6`, etc.); this dependency is a one-way street where the individual creates don't "know" about `sirin`. This reduces code interdependence, makes it so that implementors of the sub-crates don't have to know anything about `sirin`, and makes it so that those crates are reusable for other projects.

For Cargo workspaces, the profile must be [set once for the entire workspace](https://github.com/rust-lang/cargo/issues/3206). For this reason, `sirin-cli` and this book `sirin-book` are their own repositories. `uunit` is also its own repository; the implementation of that crate as generated Rust code is sufficiently complicated to warrant it.