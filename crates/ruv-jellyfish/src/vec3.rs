//! Minimal 3-D vector math.
//!
//! `ruv-jellyfish` is deliberately standalone (no dependency on the parent
//! `ruview-swarm` crate) so it can be unit-tested in isolation and reused. The
//! adapter in `ruview-swarm` maps `Position3D`/`Velocity3D` to/from [`Vec3`]
//! at the call site — see ADR-172 §Integration.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A 3-D vector in a local NED-style frame (x = north, y = east, z = down),
/// metres or metres/second depending on context.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    // Fluent method form is used throughout the crate; the operator traits
    // below mirror them for callers who prefer `+`/`-`.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    #[allow(clippy::should_implement_trait)]
    pub fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn scale(self, k: f64) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }

    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm_sq().sqrt()
    }

    pub fn distance_to(self, o: Vec3) -> f64 {
        self.sub(o).norm()
    }

    /// Unit vector, or [`Vec3::ZERO`] if this vector is (near) zero-length.
    pub fn normalized(self) -> Vec3 {
        let n = self.norm();
        if n < 1e-12 {
            Vec3::ZERO
        } else {
            self.scale(1.0 / n)
        }
    }

    /// Rescale to at most `max_len` without changing direction.
    pub fn clamped(self, max_len: f64) -> Vec3 {
        let n = self.norm();
        if n > max_len && n > 1e-12 {
            self.scale(max_len / n)
        } else {
            self
        }
    }

    /// Component of `self` along the direction of `onto`.
    /// Returns [`Vec3::ZERO`] if `onto` is (near) zero-length.
    pub fn project_onto(self, onto: Vec3) -> Vec3 {
        let d = onto.norm_sq();
        if d < 1e-12 {
            Vec3::ZERO
        } else {
            onto.scale(self.dot(onto) / d)
        }
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::add(self, o)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::sub(self, o)
    }
}

impl std::ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, k: f64) -> Vec3 {
        self.scale(k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_zero_is_zero() {
        assert_eq!(Vec3::ZERO.normalized(), Vec3::ZERO);
    }

    #[test]
    fn normalized_unit_length() {
        let v = Vec3::new(3.0, 4.0, 0.0).normalized();
        assert!((v.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn clamped_caps_length() {
        let v = Vec3::new(10.0, 0.0, 0.0).clamped(2.0);
        assert!((v.norm() - 2.0).abs() < 1e-12);
        // Already-short vectors pass through unchanged.
        let s = Vec3::new(0.5, 0.0, 0.0).clamped(2.0);
        assert_eq!(s, Vec3::new(0.5, 0.0, 0.0));
    }

    #[test]
    fn projection_recovers_parallel_component() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let axis = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(v.project_onto(axis), Vec3::new(3.0, 0.0, 0.0));
        assert_eq!(v.project_onto(Vec3::ZERO), Vec3::ZERO);
    }
}
