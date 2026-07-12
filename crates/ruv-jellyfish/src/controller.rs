//! `JellyfishController` — the per-drone façade that unifies the two behaviours
//! around a single energy budget.
//!
//! A mission typically alternates two intents, and this controller debits both
//! from one per-kilogram energy budget so a planner can reason about total
//! endurance end to end:
//!
//! * [`JellyfishController::cruise`] — efficient transit to a search area using
//!   the pulse-and-drift [`PulseDriftGait`].
//! * [`JellyfishController::loiter`] — energy-aware bloom aggregation /
//!   station keeping via [`BloomController`], priced with
//!   [`EnergyModel::station_keeping_power`].
//!
//! Both debit [`JellyfishController::budget_remaining`]; when it hits zero the
//! caller should trigger the fleet's return-to-home / failsafe.

use crate::bloom::{BloomCommand, BloomController, BloomParams};
use crate::energy::{EnergyModel, Gait};
use crate::field::{FlowField, ValueField};
use crate::pulse::{GaitParams, GaitTelemetry, PulseDriftGait};
use crate::vec3::Vec3;

/// Per-drone jellyfish behaviour controller with an energy budget.
#[derive(Debug, Clone)]
pub struct JellyfishController {
    energy: EnergyModel,
    gait: PulseDriftGait,
    bloom: BloomController,
    /// Total onboard energy budget, J·kg⁻¹.
    budget: f64,
    /// Energy debited so far, J·kg⁻¹.
    spent: f64,
    /// Gait total-energy reading at the previous cruise step, to debit deltas.
    gait_energy_mark: f64,
}

/// Result of one loiter tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoiterStep {
    /// The bloom command to fly.
    pub command: BloomCommand,
    /// Instantaneous station-keeping power at this tick, W·kg⁻¹.
    pub power: f64,
    /// Energy remaining after this tick, J·kg⁻¹.
    pub budget_remaining: f64,
}

impl JellyfishController {
    /// Build a controller with `budget` joules per kilogram of onboard energy.
    pub fn new(
        energy: EnergyModel,
        gait_params: GaitParams,
        bloom_params: BloomParams,
        budget: f64,
    ) -> Self {
        Self {
            energy,
            gait: PulseDriftGait::new(energy, gait_params),
            bloom: BloomController::new(bloom_params),
            budget: budget.max(0.0),
            spent: 0.0,
            gait_energy_mark: 0.0,
        }
    }

    /// Sensible defaults for a light coast-capable airframe.
    pub fn with_budget(budget: f64) -> Self {
        Self::new(EnergyModel::default(), GaitParams::default(), BloomParams::default(), budget)
    }

    /// Advance the transit gait by `dt`, debit the energy spent, and return the
    /// commanded speed magnitude (m·s⁻¹) along the current heading.
    pub fn cruise(&mut self, dt: f64) -> f64 {
        let speed = self.gait.step(dt);
        let total = self.gait.telemetry().total_energy();
        let delta = (total - self.gait_energy_mark).max(0.0);
        self.gait_energy_mark = total;
        self.debit(delta);
        speed
    }

    /// Compute the bloom command for a loiter/aggregation tick and debit the
    /// station-keeping energy it costs over `dt`.
    pub fn loiter<V: ValueField, F: FlowField>(
        &mut self,
        pos: Vec3,
        neighbours: &[Vec3],
        value: &V,
        flow: &F,
        t: f64,
        dt: f64,
    ) -> LoiterStep {
        let command = self.bloom.command(pos, neighbours, value, flow, t);
        let power = self
            .energy
            .station_keeping_power(Gait::PulseDrift, command.relative_flow_speed);
        self.debit(power * dt.max(0.0));
        LoiterStep { command, power, budget_remaining: self.budget_remaining() }
    }

    fn debit(&mut self, joules: f64) {
        self.spent = (self.spent + joules).min(self.budget);
    }

    /// Energy spent so far, J·kg⁻¹.
    pub fn spent(&self) -> f64 {
        self.spent
    }

    /// Energy remaining, J·kg⁻¹.
    pub fn budget_remaining(&self) -> f64 {
        (self.budget - self.spent).max(0.0)
    }

    /// Whether the budget is exhausted (trigger return-to-home upstream).
    pub fn depleted(&self) -> bool {
        self.budget_remaining() <= 0.0
    }

    /// Gait telemetry snapshot (distance, energy split, phase).
    pub fn gait_telemetry(&self) -> GaitTelemetry {
        self.gait.telemetry()
    }

    /// Estimated remaining loiter endurance (s) holding station against a
    /// relative flow of `rel_flow` (m·s⁻¹), at the current budget.
    pub fn loiter_endurance_secs(&self, rel_flow: f64) -> f64 {
        self.energy
            .loiter_endurance_secs(Gait::PulseDrift, self.budget_remaining(), rel_flow)
    }

    /// Estimated remaining cruise range (m) at average speed `v` (m·s⁻¹).
    pub fn cruise_range_metres(&self, v: f64) -> f64 {
        self.energy.range_metres(Gait::PulseDrift, self.budget_remaining(), v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::{GaussianHotspot, HotspotField, NoFlow, UniformFlow};

    #[test]
    fn cruise_drains_budget_monotonically() {
        let mut c = JellyfishController::with_budget(100_000.0);
        let mut last = c.budget_remaining();
        for _ in 0..500 {
            c.cruise(0.05);
            let now = c.budget_remaining();
            assert!(now <= last + 1e-9);
            last = now;
        }
        assert!(c.spent() > 0.0);
    }

    #[test]
    fn loiter_in_wind_costs_more_than_calm() {
        let value = HotspotField::new(vec![GaussianHotspot {
            centre: Vec3::new(100.0, 0.0, 0.0),
            peak: 1.0,
            sigma: 40.0,
        }]);
        let pos = Vec3::new(0.0, 0.0, 0.0);

        let mut calm = JellyfishController::with_budget(1_000_000.0);
        let mut windy = JellyfishController::with_budget(1_000_000.0);
        // A crosswind that the drone cannot fully exploit → higher airspeed.
        let cross = UniformFlow(Vec3::new(0.0, 6.0, 0.0));

        for _ in 0..200 {
            calm.loiter(pos, &[], &value, &NoFlow, 0.0, 0.1);
            windy.loiter(pos, &[], &value, &cross, 0.0, 0.1);
        }
        assert!(windy.spent() > calm.spent());
    }

    #[test]
    fn budget_never_goes_negative() {
        let mut c = JellyfishController::with_budget(50.0); // tiny
        for _ in 0..10_000 {
            c.cruise(0.1);
        }
        assert!(c.depleted());
        assert!(c.budget_remaining() >= 0.0);
    }

    #[test]
    fn endurance_estimate_shrinks_as_budget_drains() {
        let mut c = JellyfishController::with_budget(500_000.0);
        let e0 = c.loiter_endurance_secs(2.0);
        for _ in 0..1000 {
            c.cruise(0.05);
        }
        let e1 = c.loiter_endurance_secs(2.0);
        assert!(e1 < e0);
    }
}
