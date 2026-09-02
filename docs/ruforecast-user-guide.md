# RuForecast operator guide

RuForecast is an optional local predictive-advisory layer. It watches three
simple signals—remaining battery, connection quality, and mission progress—and
builds a receipt-bound forecast. It never steers a drone.

## Build and verify

```bash
cargo build --locked --features ruforecast
cargo test --locked --features ruforecast --all-targets
cargo test --locked --features latentmesh,ruforecast --all-targets
cargo clippy --locked --features latentmesh,ruforecast --all-targets -- -D warnings
cargo bench --locked --features ruforecast --bench ruforecast_bench
```

The feature is off by default. The `full` feature includes it, but its runtime
policy still starts in shadow mode.

## Rollout modes

| Mode | Behavior |
|---|---|
| `Disabled` | Accepts bounded history but issues no forecast |
| `Shadow` | Forecasts and measures; every eligibility query returns true |
| `CanaryReduceOnly` | A fresh low battery/link forecast may reject new work |

Use shadow until the benchmark and validation gates are met on your hardware
and mission data. Canary does not cancel existing work and does not alter a
flight path or fail-safe.

```rust
use ruview_swarm::forecast::{ForecastPolicy, ForecastRolloutMode};

let mut policy = ForecastPolicy::default();
policy.rollout = ForecastRolloutMode::CanaryReduceOnly;
policy.minimum_battery_pct = 35.0;
policy.minimum_link_quality = 0.4;

let orchestrator = orchestrator.with_forecast_policy(policy);
let may_accept_new_work =
    orchestrator.forecast_is_eligible_for_new_work(monotonic_now_ms);
```

Use a monotonic mission clock. Keep the default 128-row capacity unless a
measured need justifies more memory. Do not use forecast timestamps as wall
clock identities or mix clocks across restarts.

## What to monitor

`ForecastMetrics` exposes accepted/rejected observations, issued forecasts,
abstentions, stale inputs, invalid outputs, and canary reductions. These are
aggregate counters without raw telemetry or identifiers. Alert on increasing
rejection, staleness, invalid output, or unexpected reduction rates.

Every `ForecastAdvisory` carries model ID, origin/expiry, request digest, output
digest, low-quantile battery/link summaries, and median progress at the
horizon. Store only redacted summaries needed for your approved retention
period; do not log the bounded history by default.

## Current capability boundary

The integrated forecaster is the deterministic last-value baseline. This is
intentional: upstream RuForecast does not yet show reliable out-of-sample lift
from its learned model. The integration establishes the secure contract and
measurement path now, so a learned artifact can be compared honestly later.

Do not describe this release as predicting flight paths, autonomously rerouting
drones, or outperforming baselines. The supported benefits are safer rollout,
earlier visibility into assignment risk, reproducible forecast receipts, and a
bounded path to validated learning.
