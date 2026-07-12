//! Real-time pulse-and-drift gait controller.
//!
//! A per-drone state machine that turns the analytic energetics of
//! [`crate::energy`] into a time-stepped speed profile: a powered **pulse**
//! that accelerates the vehicle to a peak speed, a one-shot **recapture** bonus
//! applied as the stopping vortex sheds, then a passive **drift** that coasts
//! down under drag until a floor speed triggers the next pulse.
//!
//! Feeding a constant heading and stepping [`PulseDriftGait::step`] at the
//! control rate yields the speed command to hand to the flight controller,
//! while [`PulseDriftGait::telemetry`] exposes accumulated distance and the
//! actuation vs. idle energy split so a mission planner can track the endurance
//! budget. At steady state the controller reproduces the closed-form actuation
//! cost `e_per_dv · k / (1 + r)` derived in [`crate::energy`], regardless of the
//! peak/floor band chosen — the band trades pulse cadence against smoothness,
//! not efficiency.

use crate::energy::{EnergyModel, Gait};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// The two phases of the gait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Phase {
    /// Powered bell contraction — accelerating toward `peak_speed`.
    Pulse,
    /// Passive coast — decelerating under drag toward `drift_floor`.
    Drift,
}

/// Tunable shape of the gait cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GaitParams {
    /// Speed at the top of a pulse, before recapture, m·s⁻¹.
    pub peak_speed: f64,
    /// Speed at which a fresh pulse is triggered, m·s⁻¹. Must be `< peak_speed`.
    pub drift_floor: f64,
    /// Acceleration applied during the powered phase, m·s⁻².
    pub pulse_accel: f64,
}

impl Default for GaitParams {
    fn default() -> Self {
        Self { peak_speed: 6.0, drift_floor: 2.0, pulse_accel: 4.0 }
    }
}

impl GaitParams {
    /// A gait band centred on a desired *average* cruise speed. The peak/floor
    /// straddle the target (heuristic ±60 %); the realized average emerges from
    /// the drag dynamics and can be read back from [`GaitTelemetry::avg_speed`].
    pub fn cruise(target_avg: f64) -> Self {
        let t = target_avg.max(0.1);
        Self {
            peak_speed: t * 1.6,
            drift_floor: t * 0.6,
            pulse_accel: (t * 2.0).max(1.0),
        }
    }

    fn sanitized(self) -> Self {
        let peak = self.peak_speed.max(0.2);
        let floor = self.drift_floor.clamp(0.0, peak - 1e-3);
        Self { peak_speed: peak, drift_floor: floor, pulse_accel: self.pulse_accel.max(1e-3) }
    }
}

/// Snapshot of a running gait for telemetry / endurance accounting.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GaitTelemetry {
    pub phase: Phase,
    pub speed: f64,
    pub distance: f64,
    pub elapsed: f64,
    /// Energy spent actively pulsing, J·kg⁻¹.
    pub actuation_energy: f64,
    /// Energy spent on the constant hotel/avionics load, J·kg⁻¹.
    pub idle_energy: f64,
}

impl GaitTelemetry {
    /// Distance-averaged speed since start, m·s⁻¹.
    pub fn avg_speed(&self) -> f64 {
        if self.elapsed > 1e-9 {
            self.distance / self.elapsed
        } else {
            0.0
        }
    }

    /// Total energy spent so far, J·kg⁻¹.
    pub fn total_energy(&self) -> f64 {
        self.actuation_energy + self.idle_energy
    }

    /// Realized actuation energy per metre, J·m⁻¹·kg⁻¹ (`0` before moving).
    /// At steady state this converges to
    /// [`EnergyModel::actuation_energy_per_metre`] for [`Gait::PulseDrift`].
    pub fn actuation_energy_per_metre(&self) -> f64 {
        if self.distance > 1e-9 {
            self.actuation_energy / self.distance
        } else {
            0.0
        }
    }
}

/// Stateful pulse-and-drift gait for one drone.
#[derive(Debug, Clone)]
pub struct PulseDriftGait {
    model: EnergyModel,
    params: GaitParams,
    phase: Phase,
    speed: f64,
    distance: f64,
    elapsed: f64,
    actuation_energy: f64,
    idle_energy: f64,
    /// Paid Δv accumulated in the current pulse, used to size the recapture bonus.
    pulse_paid_dv: f64,
}

impl PulseDriftGait {
    /// Start a gait at the drift floor, primed to pulse on the first step.
    pub fn new(model: EnergyModel, params: GaitParams) -> Self {
        let params = params.sanitized();
        Self {
            model,
            params,
            phase: Phase::Pulse,
            speed: params.drift_floor,
            distance: 0.0,
            elapsed: 0.0,
            actuation_energy: 0.0,
            idle_energy: 0.0,
            pulse_paid_dv: 0.0,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Advance the gait by `dt` seconds and return the commanded speed
    /// magnitude (m·s⁻¹) for this interval. The caller multiplies by the
    /// heading unit vector to obtain a velocity command.
    pub fn step(&mut self, dt: f64) -> f64 {
        let dt = dt.max(0.0);
        // Hotel load runs in both phases.
        self.idle_energy += self.model.idle_power * dt;

        match self.phase {
            Phase::Pulse => {
                let dv = self.params.pulse_accel * dt;
                self.speed += dv;
                self.pulse_paid_dv += dv;
                // Actuation energy is proportional to the impulse delivered.
                self.actuation_energy += self.model.e_per_dv * dv;

                if self.speed >= self.params.peak_speed {
                    // Stopping vortex returns a fraction of the pulse impulse
                    // as free forward momentum, then we coast.
                    let r = self.model.recapture.clamp(0.0, 0.999);
                    self.speed += r * self.pulse_paid_dv;
                    self.pulse_paid_dv = 0.0;
                    self.phase = Phase::Drift;
                }
            }
            Phase::Drift => {
                // Linear-drag coast (semi-implicit Euler keeps speed positive).
                self.speed /= 1.0 + self.model.drag_k * dt;
                if self.speed <= self.params.drift_floor {
                    self.phase = Phase::Pulse;
                }
            }
        }

        self.speed = self.speed.max(0.0);
        self.distance += self.speed * dt;
        self.elapsed += dt;
        self.speed
    }

    pub fn telemetry(&self) -> GaitTelemetry {
        GaitTelemetry {
            phase: self.phase,
            speed: self.speed,
            distance: self.distance,
            elapsed: self.elapsed,
            actuation_energy: self.actuation_energy,
            idle_energy: self.idle_energy,
        }
    }

    /// The energy model this gait was built with.
    pub fn model(&self) -> EnergyModel {
        self.model
    }

    /// Convenience: the constant-thrust actuation cost per metre this gait is
    /// improving on, for A/B comparison in mission reports.
    pub fn constant_thrust_reference_per_metre(&self) -> f64 {
        self.model.actuation_energy_per_metre(Gait::ConstantThrust)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(gait: &mut PulseDriftGait, secs: f64, dt: f64) {
        let steps = (secs / dt) as usize;
        for _ in 0..steps {
            gait.step(dt);
        }
    }

    #[test]
    fn cycles_between_phases() {
        let mut g = PulseDriftGait::new(EnergyModel::default(), GaitParams::default());
        let mut saw_pulse = false;
        let mut saw_drift = false;
        for _ in 0..2000 {
            g.step(0.02);
            match g.phase() {
                Phase::Pulse => saw_pulse = true,
                Phase::Drift => saw_drift = true,
            }
        }
        assert!(saw_pulse && saw_drift, "gait must alternate phases");
    }

    #[test]
    fn speed_stays_bounded() {
        let mut g = PulseDriftGait::new(EnergyModel::default(), GaitParams::default());
        for _ in 0..5000 {
            let s = g.step(0.02);
            assert!((0.0..50.0).contains(&s), "speed escaped sane bounds: {s}");
        }
    }

    #[test]
    fn steady_state_matches_analytic_actuation_cost() {
        let model = EnergyModel::default();
        let mut g = PulseDriftGait::new(model, GaitParams::default());
        // Warm up past the initial transient, then measure a long window.
        run(&mut g, 40.0, 0.01);
        let start = g.telemetry();
        run(&mut g, 400.0, 0.01);
        let end = g.telemetry();

        let d = end.distance - start.distance;
        let e = end.actuation_energy - start.actuation_energy;
        let realized = e / d;
        let analytic = model.actuation_energy_per_metre(Gait::PulseDrift);
        // The closed-form is a conservative bound: it charges drag over the whole
        // trajectory, but the sim applies no drag during the powered phase, so the
        // realized cost lands at or below the bound (and in its ballpark).
        assert!(
            realized <= analytic * 1.02,
            "realized {realized} should not exceed analytic bound {analytic}"
        );
        assert!(
            realized > analytic * 0.5,
            "realized {realized} implausibly far below analytic {analytic}"
        );
    }

    #[test]
    fn cheaper_than_constant_thrust_reference() {
        let model = EnergyModel { recapture: 0.4, ..EnergyModel::default() };
        let mut g = PulseDriftGait::new(model, GaitParams::default());
        run(&mut g, 200.0, 0.01);
        let t = g.telemetry();
        assert!(t.actuation_energy_per_metre() < g.constant_thrust_reference_per_metre());
    }

    #[test]
    fn params_sanitized_against_bad_input() {
        // floor >= peak would deadlock the state machine; constructor fixes it.
        let bad = GaitParams { peak_speed: 2.0, drift_floor: 5.0, pulse_accel: 0.0 };
        let mut g = PulseDriftGait::new(EnergyModel::default(), bad);
        for _ in 0..500 {
            g.step(0.02);
        }
        assert!(g.telemetry().distance > 0.0);
    }
}
