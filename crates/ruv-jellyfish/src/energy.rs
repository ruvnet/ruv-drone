//! Pulse-and-drift energetics.
//!
//! Jellyfish achieve the lowest cost of transport of any measured swimmer with
//! a *pulse-and-drift* gait: a short powered bell contraction followed by a
//! long passive coast, during which the stopping vortex recaptures part of the
//! shed momentum as free forward thrust (Gemmell et al., PNAS 2013).
//!
//! We model motion along a single heading as a per-unit-mass system with
//! **linear drag** `a_drag = -k·v`. Two regimes cost energy differently:
//!
//! * **Constant thrust** holds a cruise speed `v` by continuously countering
//!   drag. The impulse it must supply to cover distance `D` is `k·D`
//!   (independent of speed under linear drag), so its actuation energy per
//!   metre is `e_per_dv · k`.
//! * **Pulse-and-drift** supplies momentum in bursts. At steady state the paid
//!   impulse per cycle plus the *freely recaptured* impulse must equal the drag
//!   loss: `(1 + r)·Δv_paid = k·v·T_cycle`. The paid actuation energy per metre
//!   works out to `e_per_dv · k / (1 + r)` — a factor `(1 + r)` cheaper than
//!   constant thrust, where `r` is the recapture fraction.
//!
//! Both regimes additionally pay a constant hotel/avionics load `p_idle`, whose
//! per-metre cost is `p_idle / v` (amortized over ground covered).
//!
//! The whole model is analytic and unit-consistent (everything is per-kilogram),
//! which keeps it cheap enough to evaluate inside a control loop and lets the
//! [`crate::bloom`] controller reason about the energy cost of fighting vs.
//! riding a flow field.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Parameters of the linear-drag pulse-and-drift energy model, per unit mass.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EnergyModel {
    /// Linear drag rate `k`, s⁻¹. A free coast decays as `v(t) = v₀·e^{-k t}`.
    pub drag_k: f64,
    /// Specific actuation cost: energy to deliver one unit of impulse (Δv),
    /// J·s·m⁻¹·kg⁻¹.
    pub e_per_dv: f64,
    /// Stopping-vortex recapture fraction `r ∈ [0, 1)`. `0` = no recapture
    /// (equivalent to constant thrust on the actuation term); higher = more of
    /// each pulse's momentum returned for free during the drift.
    pub recapture: f64,
    /// Constant hotel/avionics power draw, W·kg⁻¹.
    pub idle_power: f64,
}

impl Default for EnergyModel {
    fn default() -> Self {
        // Illustrative values for a light coast-capable airframe; tune per
        // vehicle. See ADR-172 §Calibration.
        Self {
            drag_k: 0.30,
            e_per_dv: 5.0,
            recapture: 0.35,
            idle_power: 8.0,
        }
    }
}

/// Which gait a cost is being evaluated for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Gait {
    /// Continuously powered cruise.
    ConstantThrust,
    /// Bio-inspired pulse-and-drift.
    PulseDrift,
}

impl EnergyModel {
    /// Effective recapture, clamped to a physically valid `[0, 1)`.
    fn r(&self) -> f64 {
        self.recapture.clamp(0.0, 0.999)
    }

    /// Actuation energy per metre travelled, J·m⁻¹·kg⁻¹, for the given gait.
    /// Independent of speed under linear drag.
    pub fn actuation_energy_per_metre(&self, gait: Gait) -> f64 {
        let base = self.e_per_dv * self.drag_k;
        match gait {
            Gait::ConstantThrust => base,
            Gait::PulseDrift => base / (1.0 + self.r()),
        }
    }

    /// Total energy per metre travelled at cruise speed `v` (m·s⁻¹), including
    /// the amortized idle load. `v` is clamped to a small positive floor to
    /// avoid a singularity as `v → 0`.
    pub fn energy_per_metre(&self, gait: Gait, v: f64) -> f64 {
        let v = v.max(1e-3);
        self.actuation_energy_per_metre(gait) + self.idle_power / v
    }

    /// Power required to hold station against a relative flow of speed
    /// `rel_flow` (m·s⁻¹), W·kg⁻¹. This is what a loitering drone pays to keep
    /// position; riding the flow (reducing `rel_flow`) reduces it.
    pub fn station_keeping_power(&self, gait: Gait, rel_flow: f64) -> f64 {
        let impulse_per_sec = self.drag_k * rel_flow.abs();
        let actuation = match gait {
            Gait::ConstantThrust => self.e_per_dv * impulse_per_sec,
            Gait::PulseDrift => self.e_per_dv * impulse_per_sec / (1.0 + self.r()),
        };
        actuation + self.idle_power
    }

    /// Fraction of *actuation* energy saved by pulse-and-drift relative to
    /// constant thrust: `r / (1 + r)`. Idle load is unaffected.
    pub fn actuation_saving_fraction(&self) -> f64 {
        let r = self.r();
        r / (1.0 + r)
    }

    /// Range (metres) achievable from an energy budget `budget` (J·kg⁻¹) at
    /// cruise speed `v` using `gait`.
    pub fn range_metres(&self, gait: Gait, budget: f64, v: f64) -> f64 {
        budget.max(0.0) / self.energy_per_metre(gait, v)
    }

    /// Loiter endurance (seconds) from an energy budget `budget` (J·kg⁻¹) while
    /// holding station against a relative flow of `rel_flow` (m·s⁻¹).
    pub fn loiter_endurance_secs(&self, gait: Gait, budget: f64, rel_flow: f64) -> f64 {
        let p = self.station_keeping_power(gait, rel_flow);
        if p <= 1e-9 {
            f64::INFINITY
        } else {
            budget.max(0.0) / p
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pulse_drift_never_costs_more_than_constant_thrust() {
        let m = EnergyModel::default();
        for &v in &[0.5, 1.0, 3.0, 8.0] {
            assert!(
                m.energy_per_metre(Gait::PulseDrift, v)
                    <= m.energy_per_metre(Gait::ConstantThrust, v) + 1e-12
            );
        }
    }

    #[test]
    fn saving_matches_recapture_formula() {
        let m = EnergyModel { recapture: 0.4, ..EnergyModel::default() };
        // r/(1+r) = 0.4/1.4
        assert!((m.actuation_saving_fraction() - (0.4 / 1.4)).abs() < 1e-12);
    }

    #[test]
    fn zero_recapture_equals_constant_actuation() {
        let m = EnergyModel { recapture: 0.0, ..EnergyModel::default() };
        assert!((m.actuation_saving_fraction()).abs() < 1e-12);
        assert!(
            (m.actuation_energy_per_metre(Gait::PulseDrift)
                - m.actuation_energy_per_metre(Gait::ConstantThrust))
            .abs()
                < 1e-12
        );
    }

    #[test]
    fn station_power_grows_with_relative_flow() {
        let m = EnergyModel::default();
        let calm = m.station_keeping_power(Gait::PulseDrift, 0.0);
        let breezy = m.station_keeping_power(Gait::PulseDrift, 5.0);
        assert!(breezy > calm);
        // Calm-air station keeping costs only the idle load.
        assert!((calm - m.idle_power).abs() < 1e-12);
    }

    #[test]
    fn riding_flow_extends_endurance() {
        let m = EnergyModel::default();
        let budget = 500_000.0; // J/kg
        let fighting = m.loiter_endurance_secs(Gait::PulseDrift, budget, 6.0);
        let riding = m.loiter_endurance_secs(Gait::PulseDrift, budget, 1.0);
        assert!(riding > fighting);
    }

    #[test]
    fn faster_cruise_lowers_energy_per_metre() {
        // Idle load amortizes over distance, so under linear drag faster is
        // cheaper per metre (until the airframe's own speed limit bites).
        let m = EnergyModel::default();
        assert!(
            m.energy_per_metre(Gait::PulseDrift, 8.0)
                < m.energy_per_metre(Gait::PulseDrift, 2.0)
        );
    }
}
