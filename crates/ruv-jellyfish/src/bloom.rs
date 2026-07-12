//! Bloom aggregation — decentralized, energy-aware station keeping.
//!
//! A jellyfish *bloom* (a "smack") concentrates where the medium concentrates
//! it and relaxes where it does not. This controller reproduces that at the
//! fleet level as a purely local rule each drone runs on its own observed
//! neighbours — no central solver, matching `ruv-drone`'s decentralized ethos.
//!
//! Each tick it composes four steering terms into a desired ground velocity:
//!
//! 1. **Gradient climb** toward higher [`ValueField`] value (aggregate over
//!    victim-probability / inspection-interest peaks).
//! 2. **Separation** from neighbours inside `min_spacing` (never collapse).
//! 3. **Cohesion** toward the local centroid, gated *up* by field steepness
//!    (hold the smack together only when there is something to gather around).
//! 4. **Dispersion** away from the centroid, gated up where the field is
//!    **flat** — so the fleet fans out into broad coverage when no peak
//!    dominates.
//!
//! It then makes the command **energy-aware** through honest wind
//! compensation: to hold a desired ground velocity `v` in a flow `f` the
//! vehicle must fly airspeed `v − f`, so a flow *aligned* with where the bloom
//! wants to go (a convergence zone gathering the smack) cuts the airspeed —
//! and the energy — while an opposing or cross flow raises it. The reported
//! `relative_flow_speed = |v − f|` feeds
//! [`crate::energy::EnergyModel::station_keeping_power`] so a planner can price
//! the loiter. This is where "riding the current" pays off: when the bloom
//! aggregates *with* a [`crate::field::ConvergentFlow`], station keeping
//! approaches free.

use crate::field::{FlowField, ValueField};
use crate::vec3::Vec3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Tuning for [`BloomController`]. All gains are in m·s⁻¹ (they scale unit
/// steering directions), except the dimensionless field/spacing scales.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BloomParams {
    /// Speed of the pull up the value gradient at full aggregation.
    pub grad_gain: f64,
    /// Neighbours closer than this (m) generate a separation push.
    pub min_spacing: f64,
    /// Speed of the separation push at zero spacing.
    pub separation_gain: f64,
    /// Neighbours within this radius (m) contribute to cohesion/dispersion.
    pub neighbour_radius: f64,
    /// Speed of the cohesion pull at full aggregation.
    pub cohesion_gain: f64,
    /// Speed of the coverage dispersion push where the field is flat.
    pub dispersion_gain: f64,
    /// Gradient magnitude (value·m⁻¹) that counts as "half aggregating"; the
    /// aggregation weight is `g / (g + flat_gradient)`.
    pub flat_gradient: f64,
    /// Finite-difference step handed to [`ValueField::gradient_at`], m.
    pub gradient_step: f64,
    /// Upper bound on commanded ground speed (m·s⁻¹) — station keeping is slow.
    pub max_speed: f64,
    /// Upper bound on commanded *airspeed* (m·s⁻¹). Above this the vehicle
    /// cannot fully counter the flow and is carried (drifts) with the residual.
    pub max_airspeed: f64,
}

impl Default for BloomParams {
    fn default() -> Self {
        Self {
            grad_gain: 2.0,
            min_spacing: 8.0,
            separation_gain: 3.0,
            neighbour_radius: 40.0,
            cohesion_gain: 1.0,
            dispersion_gain: 1.5,
            flat_gradient: 1e-3,
            gradient_step: 1.0,
            max_speed: 5.0,
            max_airspeed: 10.0,
        }
    }
}

/// Output of one bloom step for a single drone.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BloomCommand {
    /// What pure aggregation wants relative to the ground, before flow.
    pub desired_ground_velocity: Vec3,
    /// The airspeed the drone should actually fly to hold `desired` in the flow
    /// (`desired − flow`, clamped to `max_airspeed`). Feed this magnitude to the
    /// flight controller / gait.
    pub commanded_airspeed: Vec3,
    /// Where the drone will actually go = airspeed + flow. Equals `desired`
    /// unless the airspeed clamp saturated (then the residual flow carries it).
    pub expected_ground_velocity: Vec3,
    /// `|commanded_airspeed|`; the relative-flow speed the energy model prices.
    pub relative_flow_speed: f64,
    /// Aggregation weight ∈ [0, 1): 0 = disperse for coverage, →1 = clump.
    pub aggregation: f64,
}

/// Stateless local bloom rule. One instance can serve the whole fleet; each
/// drone calls [`BloomController::command`] with its own view of the world.
#[derive(Debug, Clone, Copy, Default)]
pub struct BloomController {
    pub params: BloomParams,
}

impl BloomController {
    pub fn new(params: BloomParams) -> Self {
        Self { params }
    }

    /// Compute this drone's command given its position, the positions of the
    /// neighbours it can see, the value field, the flow field, and time `t`.
    pub fn command<V: ValueField, F: FlowField>(
        &self,
        pos: Vec3,
        neighbours: &[Vec3],
        value: &V,
        flow: &F,
        t: f64,
    ) -> BloomCommand {
        let p = self.params;

        // --- Aggregation weight from local field steepness ---
        let grad = value.gradient_at(pos, p.gradient_step);
        let gmag = grad.norm();
        let aggr = gmag / (gmag + p.flat_gradient.max(1e-12));

        // --- 1. Gradient climb (toward higher value) ---
        let climb = grad.normalized().scale(p.grad_gain * aggr);

        // --- 2. Separation (from too-close neighbours) ---
        let mut sep = Vec3::ZERO;
        for &n in neighbours {
            let away = pos.sub(n);
            let d = away.norm();
            if d > 1e-6 && d < p.min_spacing {
                let strength = (p.min_spacing - d) / p.min_spacing;
                sep = sep.add(away.normalized().scale(strength));
            }
        }
        sep = sep.scale(p.separation_gain);

        // --- 3./4. Cohesion vs. dispersion about the local centroid ---
        let mut centroid = Vec3::ZERO;
        let mut count = 0.0;
        for &n in neighbours {
            if pos.distance_to(n) < p.neighbour_radius {
                centroid = centroid.add(n);
                count += 1.0;
            }
        }
        let (coh, disp) = if count > 0.0 {
            let centroid = centroid.scale(1.0 / count);
            let to_centroid = centroid.sub(pos).normalized();
            // Clump when steep, spread when flat.
            let coh = to_centroid.scale(p.cohesion_gain * aggr);
            let disp = to_centroid.scale(-p.dispersion_gain * (1.0 - aggr));
            (coh, disp)
        } else {
            (Vec3::ZERO, Vec3::ZERO)
        };

        // --- Compose desired ground velocity ---
        let desired = climb.add(sep).add(coh).add(disp).clamped(p.max_speed);

        // --- Energy-aware wind compensation ---
        // Airspeed needed to hold `desired` over the ground is `desired − flow`;
        // aligned flow shrinks it (cheap), opposing/cross flow grows it. Clamp
        // to what the airframe can produce; any residual carries the vehicle.
        let f = flow.flow_at(pos, t);
        let commanded_airspeed = desired.sub(f).clamped(p.max_airspeed);
        let expected_ground = commanded_airspeed.add(f);

        BloomCommand {
            desired_ground_velocity: desired,
            commanded_airspeed,
            expected_ground_velocity: expected_ground,
            relative_flow_speed: commanded_airspeed.norm(),
            aggregation: aggr,
        }
    }

    /// Integrate one drone forward by `dt` under its bloom command (Euler on the
    /// expected ground velocity). Convenience for simulation and tests.
    pub fn advance<V: ValueField, F: FlowField>(
        &self,
        pos: Vec3,
        neighbours: &[Vec3],
        value: &V,
        flow: &F,
        t: f64,
        dt: f64,
    ) -> Vec3 {
        let cmd = self.command(pos, neighbours, value, flow, t);
        pos.add(cmd.expected_ground_velocity.scale(dt))
    }
}

/// Simulate a whole fleet forward one tick with all-to-all neighbour visibility.
/// Returned positions are in the same order as `positions`. Provided for tests
/// and offline what-if analysis; the on-vehicle path uses [`BloomController::command`]
/// with each drone's locally observed neighbours.
pub fn step_fleet<V: ValueField, F: FlowField>(
    ctrl: &BloomController,
    positions: &[Vec3],
    value: &V,
    flow: &F,
    t: f64,
    dt: f64,
) -> Vec<Vec3> {
    positions
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let neighbours: Vec<Vec3> = positions
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, &q)| q)
                .collect();
            ctrl.advance(p, &neighbours, value, flow, t, dt)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::{GaussianHotspot, HotspotField, NoFlow, UniformFlow};

    fn hotspot(centre: Vec3) -> HotspotField {
        HotspotField::new(vec![GaussianHotspot { centre, peak: 1.0, sigma: 40.0 }])
    }

    #[test]
    fn helpful_tailwind_lowers_commanded_airspeed() {
        let ctrl = BloomController::default();
        // Hotspot close enough (~1σ) that the drone strongly wants to move +x.
        let value = hotspot(Vec3::new(45.0, 0.0, 0.0));
        let pos = Vec3::new(0.0, 0.0, 0.0);
        // No flow: airspeed == desired (which points at the hotspot).
        let calm = ctrl.command(pos, &[], &value, &NoFlow, 0.0);
        assert!(calm.desired_ground_velocity.x > 0.5, "drone should want to move toward hotspot");
        // Tailwind blowing toward the hotspot (+x): should assist, cutting airspeed.
        let wind = UniformFlow(Vec3::new(1.5, 0.0, 0.0));
        let windy = ctrl.command(pos, &[], &value, &wind, 0.0);
        assert!(
            windy.relative_flow_speed < calm.relative_flow_speed,
            "aligned wind should reduce required airspeed"
        );
    }

    #[test]
    fn crosswind_raises_commanded_airspeed() {
        let ctrl = BloomController::default();
        let value = hotspot(Vec3::new(45.0, 0.0, 0.0));
        let pos = Vec3::new(0.0, 0.0, 0.0);
        let calm = ctrl.command(pos, &[], &value, &NoFlow, 0.0);
        // Crosswind (+y) is orthogonal to the desired +x motion: must be countered.
        let cross = UniformFlow(Vec3::new(0.0, 4.0, 0.0));
        let windy = ctrl.command(pos, &[], &value, &cross, 0.0);
        assert!(windy.relative_flow_speed > calm.relative_flow_speed);
    }

    #[test]
    fn aggregates_toward_hotspot() {
        let ctrl = BloomController::default();
        let centre = Vec3::new(150.0, 150.0, 0.0);
        let value = hotspot(centre);
        let mut pos = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(300.0, 0.0, 0.0),
            Vec3::new(0.0, 300.0, 0.0),
            Vec3::new(300.0, 300.0, 0.0),
        ];
        let spread0 = mean_dist_to(&pos, centre);
        for _ in 0..600 {
            pos = step_fleet(&ctrl, &pos, &value, &NoFlow, 0.0, 0.2);
        }
        let spread1 = mean_dist_to(&pos, centre);
        assert!(spread1 < spread0, "fleet should contract toward the hotspot");
    }

    #[test]
    fn respects_minimum_spacing() {
        let ctrl = BloomController::default();
        let value = hotspot(Vec3::new(0.0, 0.0, 0.0)); // peak right where they gather
        let mut pos = vec![
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, -2.0, 0.0),
        ];
        for _ in 0..800 {
            pos = step_fleet(&ctrl, &pos, &value, &NoFlow, 0.0, 0.1);
        }
        // No pair should be crushed to (near) zero separation.
        for i in 0..pos.len() {
            for j in (i + 1)..pos.len() {
                assert!(
                    pos[i].distance_to(pos[j]) > 1.0,
                    "drones collapsed: {:?} vs {:?}",
                    pos[i],
                    pos[j]
                );
            }
        }
    }

    #[test]
    fn disperses_when_field_is_flat() {
        // Flat field (all zero value everywhere) → dispersion should dominate
        // and spread a tight cluster out for coverage.
        struct Flat;
        impl ValueField for Flat {
            fn value_at(&self, _p: Vec3) -> f64 {
                0.0
            }
        }
        let ctrl = BloomController::default();
        let mut pos = vec![
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        ];
        let spread0 = pairwise_spread(&pos);
        for _ in 0..300 {
            pos = step_fleet(&ctrl, &pos, &Flat, &NoFlow, 0.0, 0.2);
        }
        let spread1 = pairwise_spread(&pos);
        assert!(spread1 > spread0, "flat field should disperse the cluster");
    }

    fn mean_dist_to(pos: &[Vec3], c: Vec3) -> f64 {
        pos.iter().map(|p| p.distance_to(c)).sum::<f64>() / pos.len() as f64
    }

    fn pairwise_spread(pos: &[Vec3]) -> f64 {
        let mut s = 0.0_f64;
        let mut n = 0.0_f64;
        for i in 0..pos.len() {
            for j in (i + 1)..pos.len() {
                s += pos[i].distance_to(pos[j]);
                n += 1.0;
            }
        }
        s / n.max(1.0)
    }
}
