fn main() {
    // Determinism tips at build time (advisory).
    println!("cargo:warning=Determinism: pin toolchain and consider RUSTFLAGS='-C target-feature=-fma' on all machines.");
}
