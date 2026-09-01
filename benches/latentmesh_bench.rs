use criterion::{black_box, criterion_group, criterion_main, Criterion};
use ed25519_dalek::SigningKey;
use ruview_swarm::{
    failsafe::FailSafeState,
    latentmesh::{
        to_critical_state, LatentMeshRxSession, LatentMeshTxSession, RxConfig, TrustedPeerKeys,
        TxConfig,
    },
    DroneState, NodeId, Position3D, Velocity3D,
};

const SOURCE: u32 = 7;
const NOW_MS: u64 = 10_000;

fn state() -> DroneState {
    DroneState {
        id: NodeId(SOURCE),
        position: Position3D {
            x: 12.5,
            y: -4.0,
            z: -30.0,
        },
        velocity: Velocity3D {
            vx: 2.0,
            vy: 0.5,
            vz: 0.0,
        },
        heading_rad: 0.25,
        altitude_agl_m: 30.0,
        battery_pct: 82.0,
        link_quality: 0.9,
        timestamp_ms: NOW_MS,
    }
}

fn latentmesh_benchmarks(c: &mut Criterion) {
    let state = state();
    c.bench_function("latentmesh/state_projection", |b| {
        b.iter(|| to_critical_state(black_box(&state), black_box(&FailSafeState::Nominal)))
    });

    c.bench_function("latentmesh/sign_and_fragment_64b_mtu", |b| {
        b.iter_batched(
            || {
                LatentMeshTxSession::new(
                    SOURCE,
                    1,
                    SigningKey::from_bytes(&[7; 32]),
                    TxConfig {
                        frame_mtu: 64,
                        ..TxConfig::default()
                    },
                )
                .unwrap()
            },
            |mut tx| {
                black_box(
                    tx.encode_advisory_state(&state, &FailSafeState::Nominal)
                        .unwrap(),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    let signing_key = SigningKey::from_bytes(&[7; 32]);
    let mut tx = LatentMeshTxSession::new(
        SOURCE,
        1,
        signing_key.clone(),
        TxConfig {
            frame_mtu: 64,
            ..TxConfig::default()
        },
    )
    .unwrap();
    let frames = tx
        .encode_advisory_state(&state, &FailSafeState::Nominal)
        .unwrap()
        .encoded_frames()
        .unwrap();

    c.bench_function("latentmesh/reassemble_verify_admit_64b_mtu", |b| {
        b.iter_batched(
            || {
                let mut keys = TrustedPeerKeys::new();
                keys.insert(SOURCE, signing_key.verifying_key());
                LatentMeshRxSession::new(keys, RxConfig::default()).unwrap()
            },
            |mut rx| {
                for frame in &frames {
                    black_box(rx.ingest_frame_bytes(SOURCE, NOW_MS, frame).unwrap());
                }
            },
            criterion::BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, latentmesh_benchmarks);
criterion_main!(benches);
