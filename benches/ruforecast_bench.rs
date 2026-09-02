use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use ruview_swarm::forecast::{ForecastEngine, ForecastPolicy, ForecastRolloutMode};

fn primed_engine(rollout: ForecastRolloutMode) -> ForecastEngine {
    let policy = ForecastPolicy {
        rollout,
        ..ForecastPolicy::default()
    };
    let mut engine = ForecastEngine::new(policy);
    for step in 1..=128 {
        engine.observe_and_forecast(
            step * 1_000,
            100.0 - step as f32 * 0.1,
            0.9,
            step as f32 / 128.0,
        );
    }
    engine
}

fn ruforecast_benchmarks(c: &mut Criterion) {
    c.bench_function("ruforecast/observe_cadence_limited_128_rows", |b| {
        b.iter_batched(
            || primed_engine(ForecastRolloutMode::Shadow),
            |mut engine| {
                engine.observe_and_forecast(128_100, 87.1, 0.9, 1.0);
                black_box(engine.history_len());
            },
            BatchSize::SmallInput,
        )
    });

    c.bench_function("ruforecast/observe_and_forecast_128_rows", |b| {
        b.iter_batched(
            || primed_engine(ForecastRolloutMode::Shadow),
            |mut engine| {
                engine.observe_and_forecast(129_000, 87.1, 0.9, 1.0);
                black_box(engine.last_advisory());
            },
            BatchSize::SmallInput,
        )
    });

    let engine = primed_engine(ForecastRolloutMode::CanaryReduceOnly);
    c.bench_function("ruforecast/reduce_only_policy", |b| {
        b.iter(|| black_box(engine.is_eligible_for_new_work(black_box(128_000))))
    });
}

criterion_group!(benches, ruforecast_benchmarks);
criterion_main!(benches);
