fn main() {
    // Ensure rebuild if this file changes.
    println!("cargo:rerun-if-changed=build.rs");

    // Determinism tips at build time (advisory).
    println!("cargo:warning=Determinism: pin toolchain + dependency versions; consider disabling FMA globally with RUSTFLAGS='-C target-feature=-fma' on all machines.");
}
