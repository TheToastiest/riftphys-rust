// Scalar is currently f32. If you enable `fixed`, we hard-fail to avoid a silent “not actually fixed” build.

#[cfg(feature = "fixed")]
compile_error!("feature `fixed` is not implemented yet. Scalar is f32 only. Disable `fixed` or implement a fixed-point Scalar backend.");

pub type Scalar = f32;
