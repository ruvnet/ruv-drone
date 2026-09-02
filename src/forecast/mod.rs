//! RuForecast predictive advisory integration.
//!
//! This module is intentionally outside flight authority. It consumes only
//! battery, link-quality, and aggregate mission-progress observations. Its one
//! policy output is a reduce-only eligibility result for *new* cooperative
//! work. The default rollout is shadow mode, where even that result is ignored.

use ruforecast_core::{
    CanonicalDigest, DataPolicy, FeatureSchema, FeatureSpec, ForecastOutcome, ForecastRequest,
    Forecaster, LastValueForecaster, PrivacyClass, QuantileSet, SourceState, TimeSeries,
};
use std::collections::VecDeque;

const VARIATES: usize = 3;
const BATTERY: usize = 0;
const LINK: usize = 1;
const PROGRESS: usize = 2;

/// Deployment stage. Promotion from shadow requires independent evidence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ForecastRolloutMode {
    Disabled,
    #[default]
    Shadow,
    CanaryReduceOnly,
}

/// Bounded local policy. Values are validated by [`ForecastEngine::try_new`].
#[derive(Clone, Debug)]
pub struct ForecastPolicy {
    pub rollout: ForecastRolloutMode,
    pub history_capacity: usize,
    pub minimum_history: usize,
    pub max_input_age_ms: u64,
    pub horizon: usize,
    pub step_ms: u64,
    pub advisory_ttl_ms: u64,
    pub minimum_battery_pct: f32,
    pub minimum_link_quality: f32,
}

impl Default for ForecastPolicy {
    fn default() -> Self {
        Self {
            rollout: ForecastRolloutMode::Shadow,
            history_capacity: 128,
            minimum_history: 8,
            max_input_age_ms: 5_000,
            horizon: 6,
            step_ms: 1_000,
            advisory_ttl_ms: 5_000,
            minimum_battery_pct: 30.0,
            minimum_link_quality: 0.35,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Observation {
    timestamp_ms: u64,
    values: [f32; VARIATES],
}

/// Receipt-bound, expiring forecast summary. It contains no position,
/// velocity, identity, raw history, or model weights.
#[derive(Clone, Debug, PartialEq)]
pub struct ForecastAdvisory {
    pub origin_ms: u64,
    pub expires_at_ms: u64,
    pub model_id: &'static str,
    pub request_digest: CanonicalDigest,
    pub output_digest: CanonicalDigest,
    pub minimum_battery_pct: f32,
    pub minimum_link_quality: f32,
    pub progress_at_horizon: f32,
}

/// Payload-free operational counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ForecastMetrics {
    pub observations_accepted: u64,
    pub observations_rejected: u64,
    pub forecasts_issued: u64,
    pub abstentions: u64,
    pub stale_inputs: u64,
    pub invalid_outputs: u64,
    pub eligibility_reductions: u64,
}

/// Bounded forecast state owned by one orchestrator.
pub struct ForecastEngine {
    policy: ForecastPolicy,
    history: VecDeque<Observation>,
    schema: FeatureSchema,
    quantiles: QuantileSet,
    forecaster: LastValueForecaster,
    last_advisory: Option<ForecastAdvisory>,
    last_forecast_at_ms: Option<u64>,
    metrics: ForecastMetrics,
}

impl Default for ForecastEngine {
    fn default() -> Self {
        Self::new(ForecastPolicy::default())
    }
}

impl ForecastEngine {
    /// Construct an engine, falling back to a disabled safe configuration if a
    /// caller supplies invalid bounds. Use [`Self::try_new`] to surface errors.
    #[must_use]
    pub fn new(policy: ForecastPolicy) -> Self {
        Self::try_new(policy).unwrap_or_else(|_| {
            let safe = ForecastPolicy {
                rollout: ForecastRolloutMode::Disabled,
                ..ForecastPolicy::default()
            };
            Self::try_new(safe).expect("built-in forecast policy is valid")
        })
    }

    pub fn try_new(policy: ForecastPolicy) -> Result<Self, &'static str> {
        if policy.history_capacity == 0 || policy.history_capacity > 16_384 {
            return Err("history_capacity must be in 1..=16384");
        }
        if policy.minimum_history == 0 || policy.minimum_history > policy.history_capacity {
            return Err("minimum_history must fit within history_capacity");
        }
        if policy.horizon == 0 || policy.step_ms == 0 || policy.advisory_ttl_ms == 0 {
            return Err("horizon, step_ms, and advisory_ttl_ms must be nonzero");
        }
        if policy.horizon > ruforecast_core::MAX_HORIZON
            || policy.step_ms > ruforecast_core::MAX_STEP_MS
            || u64::try_from(policy.horizon)
                .ok()
                .and_then(|horizon| horizon.checked_mul(policy.step_ms))
                .is_none_or(|span| span > ruforecast_core::MAX_FORECAST_SPAN_MS)
        {
            return Err("forecast horizon or cadence exceeds RuForecast bounds");
        }
        if !policy.minimum_battery_pct.is_finite()
            || !(0.0..=100.0).contains(&policy.minimum_battery_pct)
            || !policy.minimum_link_quality.is_finite()
            || !(0.0..=1.0).contains(&policy.minimum_link_quality)
        {
            return Err("forecast thresholds are outside physical ranges");
        }
        let schema = FeatureSchema::new(vec![
            FeatureSpec::new("battery_pct", "percent").map_err(|_| "invalid schema")?,
            FeatureSpec::new("link_quality", "ratio").map_err(|_| "invalid schema")?,
            FeatureSpec::new("mission_progress", "ratio").map_err(|_| "invalid schema")?,
        ])
        .map_err(|_| "invalid schema")?;
        let quantiles = QuantileSet::new(vec![0.1, 0.5, 0.9]).map_err(|_| "invalid quantiles")?;
        Ok(Self {
            history: VecDeque::with_capacity(policy.history_capacity),
            policy,
            schema,
            quantiles,
            forecaster: LastValueForecaster::new(),
            last_advisory: None,
            last_forecast_at_ms: None,
            metrics: ForecastMetrics::default(),
        })
    }

    /// Admit one bounded observation and refresh the shadow advisory. Invalid,
    /// duplicate, or non-monotonic samples fail closed and never replace the
    /// last valid advisory.
    pub fn observe_and_forecast(
        &mut self,
        timestamp_ms: u64,
        battery_pct: f32,
        link_quality: f32,
        mission_progress: f32,
    ) {
        if !battery_pct.is_finite()
            || !(0.0..=100.0).contains(&battery_pct)
            || !link_quality.is_finite()
            || !(0.0..=1.0).contains(&link_quality)
            || !mission_progress.is_finite()
            || !(0.0..=1.0).contains(&mission_progress)
            || self
                .history
                .back()
                .is_some_and(|last| timestamp_ms <= last.timestamp_ms)
        {
            self.metrics.observations_rejected =
                self.metrics.observations_rejected.saturating_add(1);
            return;
        }
        if self.history.len() == self.policy.history_capacity {
            self.history.pop_front();
        }
        self.history.push_back(Observation {
            timestamp_ms,
            values: [battery_pct, link_quality, mission_progress],
        });
        self.metrics.observations_accepted = self.metrics.observations_accepted.saturating_add(1);
        if self
            .last_forecast_at_ms
            .is_none_or(|last| timestamp_ms.saturating_sub(last) >= self.policy.step_ms)
        {
            self.refresh(timestamp_ms);
        }
    }

    fn refresh(&mut self, now_ms: u64) {
        if self.policy.rollout == ForecastRolloutMode::Disabled
            || self.history.len() < self.policy.minimum_history
        {
            self.metrics.abstentions = self.metrics.abstentions.saturating_add(1);
            return;
        }
        let Some(last) = self.history.back() else {
            return;
        };
        if now_ms.saturating_sub(last.timestamp_ms) > self.policy.max_input_age_ms {
            self.metrics.stale_inputs = self.metrics.stale_inputs.saturating_add(1);
            return;
        }
        let Some(series) = self.materialize_series() else {
            self.metrics.invalid_outputs = self.metrics.invalid_outputs.saturating_add(1);
            return;
        };
        let Ok(request) = ForecastRequest::new(
            &series,
            self.policy.horizon,
            self.policy.step_ms,
            &self.quantiles,
        ) else {
            self.metrics.invalid_outputs = self.metrics.invalid_outputs.saturating_add(1);
            return;
        };
        let request_digest = request.canonical_digest();
        match self.forecaster.forecast(&request) {
            Ok(ForecastOutcome::Forecast(forecast))
                if forecast.verify_payload_integrity().is_ok() =>
            {
                let last_step = forecast.horizon() - 1;
                let low = 0;
                let median = 1;
                let (
                    Some(minimum_battery_pct),
                    Some(minimum_link_quality),
                    Some(progress_at_horizon),
                ) = (
                    (0..forecast.horizon())
                        .filter_map(|step| forecast.value(step, BATTERY, low))
                        .reduce(f32::min),
                    (0..forecast.horizon())
                        .filter_map(|step| forecast.value(step, LINK, low))
                        .reduce(f32::min),
                    forecast.value(last_step, PROGRESS, median),
                )
                else {
                    self.metrics.invalid_outputs = self.metrics.invalid_outputs.saturating_add(1);
                    return;
                };
                self.last_advisory = Some(ForecastAdvisory {
                    origin_ms: forecast.origin_ms(),
                    expires_at_ms: forecast
                        .origin_ms()
                        .saturating_add(self.policy.advisory_ttl_ms),
                    model_id: "ruview-last-value-baseline",
                    request_digest,
                    output_digest: forecast.receipt().output_digest(),
                    minimum_battery_pct,
                    minimum_link_quality,
                    progress_at_horizon,
                });
                self.last_forecast_at_ms = Some(now_ms);
                self.metrics.forecasts_issued = self.metrics.forecasts_issued.saturating_add(1);
                if self.policy.rollout == ForecastRolloutMode::CanaryReduceOnly
                    && (minimum_battery_pct < self.policy.minimum_battery_pct
                        || minimum_link_quality < self.policy.minimum_link_quality)
                {
                    self.metrics.eligibility_reductions =
                        self.metrics.eligibility_reductions.saturating_add(1);
                }
            }
            Ok(ForecastOutcome::Abstained(_)) => {
                self.metrics.abstentions = self.metrics.abstentions.saturating_add(1);
            }
            _ => {
                self.metrics.invalid_outputs = self.metrics.invalid_outputs.saturating_add(1);
            }
        }
    }

    fn materialize_series(&self) -> Option<TimeSeries> {
        let mut timestamps = Vec::with_capacity(self.history.len());
        let mut values = Vec::with_capacity(self.history.len() * VARIATES);
        for observation in &self.history {
            timestamps.push(observation.timestamp_ms);
            values.extend_from_slice(&observation.values);
        }
        let retention_until_ms = timestamps
            .last()?
            .saturating_add(self.policy.advisory_ttl_ms);
        let policy = DataPolicy::new(
            PrivacyClass::P1,
            "local-drone",
            "local-node",
            "mission-runtime",
            "local predictive advisory",
            CanonicalDigest::of_bytes(b"ruv-drone-ruforecast-policy-v1", b"local-only"),
            None,
            None,
            None,
            retention_until_ms,
            true,
        )
        .ok()?;
        TimeSeries::new(
            self.schema.clone(),
            timestamps,
            values,
            vec![true; self.history.len() * VARIATES],
            SourceState::claimed("bounded local drone telemetry").ok()?,
            policy,
        )
        .ok()
    }

    /// Return `false` only for a fresh canary advisory below a local threshold.
    /// Every other state preserves existing assignment behavior.
    pub fn is_eligible_for_new_work(&self, now_ms: u64) -> bool {
        if self.policy.rollout != ForecastRolloutMode::CanaryReduceOnly {
            return true;
        }
        self.last_advisory.as_ref().is_none_or(|advisory| {
            now_ms > advisory.expires_at_ms
                || (advisory.minimum_battery_pct >= self.policy.minimum_battery_pct
                    && advisory.minimum_link_quality >= self.policy.minimum_link_quality)
        })
    }

    #[must_use]
    pub fn last_advisory(&self) -> Option<&ForecastAdvisory> {
        self.last_advisory.as_ref()
    }

    #[must_use]
    pub const fn metrics(&self) -> ForecastMetrics {
        self.metrics
    }

    #[must_use]
    pub fn history_len(&self) -> usize {
        self.history.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(engine: &mut ForecastEngine, battery: f32, link: f32) {
        for step in 1..=8 {
            engine.observe_and_forecast(step * 1_000, battery, link, step as f32 / 10.0);
        }
    }

    #[test]
    fn default_is_shadow_and_never_reduces_eligibility() {
        let mut engine = ForecastEngine::default();
        feed(&mut engine, 5.0, 0.01);
        assert!(engine.last_advisory().is_some());
        assert!(engine.is_eligible_for_new_work(8_000));
    }

    #[test]
    fn canary_is_reduce_only_and_stale_advice_is_ignored() {
        let policy = ForecastPolicy {
            rollout: ForecastRolloutMode::CanaryReduceOnly,
            ..ForecastPolicy::default()
        };
        let mut engine = ForecastEngine::new(policy);
        feed(&mut engine, 20.0, 0.2);
        assert!(!engine.is_eligible_for_new_work(8_000));
        assert!(engine.is_eligible_for_new_work(13_001));
    }

    #[test]
    fn history_is_bounded_and_non_monotonic_input_is_rejected() {
        let policy = ForecastPolicy {
            history_capacity: 8,
            minimum_history: 2,
            ..ForecastPolicy::default()
        };
        let mut engine = ForecastEngine::new(policy);
        for step in 1..=20 {
            engine.observe_and_forecast(step, 90.0, 0.9, 0.5);
        }
        engine.observe_and_forecast(20, 90.0, 0.9, 0.5);
        assert_eq!(engine.history_len(), 8);
        assert_eq!(engine.metrics().observations_rejected, 1);
    }

    #[test]
    fn nonfinite_and_out_of_range_observations_never_replace_receipt() {
        let mut engine = ForecastEngine::default();
        feed(&mut engine, 90.0, 0.9);
        let digest = engine.last_advisory().unwrap().output_digest;
        engine.observe_and_forecast(9_000, f32::NAN, 0.9, 0.9);
        engine.observe_and_forecast(10_000, 90.0, 2.0, 0.9);
        assert_eq!(engine.last_advisory().unwrap().output_digest, digest);
        assert_eq!(engine.metrics().observations_rejected, 2);
    }

    #[test]
    fn invalid_policy_fails_disabled() {
        let policy = ForecastPolicy {
            history_capacity: 0,
            ..ForecastPolicy::default()
        };
        let engine = ForecastEngine::new(policy);
        assert!(engine.is_eligible_for_new_work(u64::MAX));
        assert!(engine.last_advisory().is_none());
    }

    #[test]
    fn oversized_horizon_fails_disabled() {
        let policy = ForecastPolicy {
            horizon: ruforecast_core::MAX_HORIZON + 1,
            rollout: ForecastRolloutMode::CanaryReduceOnly,
            ..ForecastPolicy::default()
        };
        let engine = ForecastEngine::new(policy);
        assert!(engine.is_eligible_for_new_work(0));
    }
}
