#[cfg(feature = "fixed")]
pub type Scalar = f32; // placeholder for a fixed-point backend later

#[cfg(not(feature = "fixed"))]
pub type Scalar = f32;
