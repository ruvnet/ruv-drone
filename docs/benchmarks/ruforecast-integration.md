# RuForecast integration benchmark and validation protocol

## Questions

1. What CPU time and allocation cost does one 128-row observation/forecast add?
2. What does the reduce-only policy query cost?
3. Does cadence limiting prevent forecast construction on every control tick?
4. Can a candidate beat both last-value and seasonal-naive on unseen data?

## Reproducer

```bash
cargo bench --locked --features ruforecast --bench ruforecast_bench
cargo test --locked --features ruforecast --all-targets
cargo test --locked --features latentmesh,ruforecast --all-targets
```

Record CPU, OS, Rust version, git commit, build profile, p50 estimate and
Criterion confidence interval. Results are software measurements, not radio,
energy, prediction-accuracy, or end-to-end flight claims.

## Performance gates

The initial engineering budgets on representative companion hardware are:

| Operation | Gate |
|---|---:|
| Reduce-only eligibility query | p99 ≤ 10 µs |
| 128-row baseline refresh | p99 ≤ 1 ms |
| Forecast history | ≤ 128 rows by default |
| Added control-tick work before cadence | range check + bounded queue only |

Criterion measures central tendency and confidence intervals, not p99. A
target-hardware load harness must verify p99 before canary promotion.

## Accuracy gate for a learned candidate

Use temporally separated, out-of-search corpora with frozen preprocessing.
Report weighted quantile loss overall and at each horizon, 80% interval
coverage, missingness slices, calibration, abstention rate, and both baseline
scores. Predeclare margins and seeds. A candidate fails if it loses to either
baseline on any required corpus, even when its average looks better.

## Optimization record

The first code-level optimization is cadence limiting: the bounded queue is
updated at observation rate, but series materialization, canonical hashing,
and inference run no more frequently than `step_ms`. This avoids repeated heap
allocation and SHA-256 work when the control loop runs faster than the forecast
cadence. Further optimization requires profiler or benchmark evidence; do not
weaken validation, receipts, bounds, or the flight-authority separation.

## Branch measurements

Measured 2026-09-02 on an x86_64 Intel Xeon Platinum 8573C runner with
`rustc 1.98.0`, optimized Criterion profile, 1-second warm-up, 2-second
measurement, and 30 samples:

| Operation | Criterion estimate (95% confidence interval) |
|---|---:|
| Cadence-limited observation, 128-row queue | 183.96–235.14 ns |
| Observation plus 128-row baseline refresh | 11.385–12.003 µs |
| Fresh reduce-only eligibility query | 690.74–764.99 ps |

The policy query is small enough for the optimizer to reduce to a few loads and
comparisons; treat the sub-nanosecond result as a microbenchmark lower bound,
not an end-to-end latency claim. Cadence limiting makes the usual observation
path roughly 50 times cheaper than a refresh on this runner. These values
should not be copied to other hardware as a guarantee.
