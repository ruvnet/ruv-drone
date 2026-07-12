//! Flow and value fields — the "medium" a bloom rides and the "prey density"
//! it aggregates over.
//!
//! * A [`FlowField`] is the wind/current vector at a point and time. Riding it
//!   (instead of fighting it) is what makes station keeping cheap — see
//!   [`crate::energy::EnergyModel::station_keeping_power`].
//! * A [`ValueField`] is a scalar map of where the fleet's cooperative
//!   attention is worth spending: SAR victim probability, inspection interest,
//!   NDVI anomaly. The [`crate::bloom`] controller climbs its gradient to
//!   densify a smack over high-value regions and relaxes to broad coverage
//!   where it is flat.
//!
//! Both are traits so a mission can plug in an analytic model, a sampled grid,
//! or a live estimate (e.g. the Bayesian probability grid from `ruv-drone`'s
//! `planning::probability_grid`). Default implementations for the common cases
//! ship here.

use crate::vec3::Vec3;

/// A time-varying vector field: wind aloft, water current, thermal drift.
pub trait FlowField {
    /// Flow velocity (m·s⁻¹) at position `p` and time `t` (seconds).
    fn flow_at(&self, p: Vec3, t: f64) -> Vec3;
}

/// A scalar field of cooperative interest. Higher = more worth loitering over.
pub trait ValueField {
    /// Value at position `p`.
    fn value_at(&self, p: Vec3) -> f64;

    /// Spatial gradient of the value (points toward increasing value). The
    /// default is a central finite difference with step `h`; override it when
    /// an analytic gradient is available.
    fn gradient_at(&self, p: Vec3, h: f64) -> Vec3 {
        let h = h.max(1e-6);
        let dx = self.value_at(Vec3::new(p.x + h, p.y, p.z))
            - self.value_at(Vec3::new(p.x - h, p.y, p.z));
        let dy = self.value_at(Vec3::new(p.x, p.y + h, p.z))
            - self.value_at(Vec3::new(p.x, p.y - h, p.z));
        // Value fields are treated as 2-D over the ground plane; z (altitude)
        // is handled separately by the flight controller, so no z-gradient.
        Vec3::new(dx / (2.0 * h), dy / (2.0 * h), 0.0)
    }
}

/// Still air / no current.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoFlow;

impl FlowField for NoFlow {
    fn flow_at(&self, _p: Vec3, _t: f64) -> Vec3 {
        Vec3::ZERO
    }
}

/// A spatially uniform, constant flow.
#[derive(Debug, Clone, Copy)]
pub struct UniformFlow(pub Vec3);

impl FlowField for UniformFlow {
    fn flow_at(&self, _p: Vec3, _t: f64) -> Vec3 {
        self.0
    }
}

/// A flow that converges toward (or diverges from) a centre point, modelling a
/// thermocline / convergence zone that passively concentrates a bloom. Positive
/// `strength` pulls inward; the pull saturates near the centre to stay finite.
#[derive(Debug, Clone, Copy)]
pub struct ConvergentFlow {
    pub centre: Vec3,
    /// Peak inward speed, m·s⁻¹.
    pub strength: f64,
    /// Distance scale over which the pull ramps up, m.
    pub scale: f64,
}

impl FlowField for ConvergentFlow {
    fn flow_at(&self, p: Vec3, _t: f64) -> Vec3 {
        let to_centre = self.centre.sub(p);
        let d = to_centre.norm();
        if d < 1e-6 {
            return Vec3::ZERO;
        }
        // Saturating radial profile: ~linear near centre, flat far out.
        let speed = self.strength * (d / (d + self.scale.max(1e-6)));
        to_centre.scale(speed / d)
    }
}

/// A single isotropic Gaussian "hot spot" of interest — the simplest useful
/// [`ValueField`], with an analytic gradient.
#[derive(Debug, Clone, Copy)]
pub struct GaussianHotspot {
    pub centre: Vec3,
    /// Peak value at the centre.
    pub peak: f64,
    /// Standard deviation (m).
    pub sigma: f64,
}

impl ValueField for GaussianHotspot {
    fn value_at(&self, p: Vec3) -> f64 {
        let d2 = self.centre.sub(p).norm_sq();
        let s2 = self.sigma.max(1e-6).powi(2);
        self.peak * (-d2 / (2.0 * s2)).exp()
    }

    fn gradient_at(&self, p: Vec3, _h: f64) -> Vec3 {
        // ∇ = value(p) · (centre − p) / σ²  (points uphill, toward the centre).
        let s2 = self.sigma.max(1e-6).powi(2);
        let v = self.value_at(p);
        let to_centre = self.centre.sub(p);
        Vec3::new(to_centre.x, to_centre.y, 0.0).scale(v / s2)
    }
}

/// Sum of several [`GaussianHotspot`]s — enough to represent a multi-modal
/// probability map without pulling in a full grid sampler.
#[derive(Debug, Clone, Default)]
pub struct HotspotField {
    pub hotspots: Vec<GaussianHotspot>,
}

impl HotspotField {
    pub fn new(hotspots: Vec<GaussianHotspot>) -> Self {
        Self { hotspots }
    }

    pub fn push(&mut self, h: GaussianHotspot) {
        self.hotspots.push(h);
    }
}

impl ValueField for HotspotField {
    fn value_at(&self, p: Vec3) -> f64 {
        self.hotspots.iter().map(|h| h.value_at(p)).sum()
    }

    fn gradient_at(&self, p: Vec3, h: f64) -> Vec3 {
        self.hotspots
            .iter()
            .fold(Vec3::ZERO, |acc, hs| acc.add(hs.gradient_at(p, h)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_flow_is_constant() {
        let f = UniformFlow(Vec3::new(1.0, -2.0, 0.0));
        assert_eq!(f.flow_at(Vec3::new(5.0, 5.0, 0.0), 3.0), Vec3::new(1.0, -2.0, 0.0));
    }

    #[test]
    fn convergent_flow_points_inward() {
        let f = ConvergentFlow { centre: Vec3::ZERO, strength: 2.0, scale: 10.0 };
        let v = f.flow_at(Vec3::new(20.0, 0.0, 0.0), 0.0);
        assert!(v.x < 0.0, "flow east of centre should push west");
        assert!(f.flow_at(Vec3::ZERO, 0.0).norm() < 1e-9, "no pull at the centre");
    }

    #[test]
    fn gaussian_gradient_points_uphill() {
        let g = GaussianHotspot { centre: Vec3::new(100.0, 0.0, 0.0), peak: 1.0, sigma: 30.0 };
        let grad = g.gradient_at(Vec3::new(50.0, 0.0, 0.0), 1.0);
        assert!(grad.x > 0.0, "gradient should point toward the hotspot centre");
    }

    #[test]
    fn analytic_and_numeric_gradient_agree() {
        let g = GaussianHotspot { centre: Vec3::new(0.0, 0.0, 0.0), peak: 2.0, sigma: 25.0 };
        let p = Vec3::new(12.0, -7.0, 0.0);
        let analytic = g.gradient_at(p, 1.0);
        // Compare against the default central-difference via a bare closure field.
        struct Wrap(GaussianHotspot);
        impl ValueField for Wrap {
            fn value_at(&self, p: Vec3) -> f64 {
                self.0.value_at(p)
            }
        }
        let numeric = Wrap(g).gradient_at(p, 0.5);
        assert!((analytic.x - numeric.x).abs() < 1e-3);
        assert!((analytic.y - numeric.y).abs() < 1e-3);
    }

    #[test]
    fn hotspot_field_superposes() {
        let field = HotspotField::new(vec![
            GaussianHotspot { centre: Vec3::new(-50.0, 0.0, 0.0), peak: 1.0, sigma: 20.0 },
            GaussianHotspot { centre: Vec3::new(50.0, 0.0, 0.0), peak: 1.0, sigma: 20.0 },
        ]);
        // Midpoint sits between two symmetric peaks: gradient ~cancels.
        let grad = field.gradient_at(Vec3::ZERO, 1.0);
        assert!(grad.norm() < 1e-6);
    }
}
