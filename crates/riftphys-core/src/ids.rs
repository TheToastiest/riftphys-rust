use core::fmt;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(pub u32);
impl fmt::Display for BodyId { fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "BodyId({})", self.0) } }

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct ColliderId(pub u32);
impl fmt::Display for ColliderId { fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "ColliderId({})", self.0) } }

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct JointId(pub u32);
impl fmt::Display for JointId { fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "JointId({})", self.0) } }
