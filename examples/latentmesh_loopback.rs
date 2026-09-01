use std::error::Error;

use ed25519_dalek::SigningKey;
use ruview_swarm::{
    failsafe::FailSafeState,
    latentmesh::{
        bounded_channel_loopback, AdaptivePolicy, FrameTransport, LatentMeshNode,
        LatentMeshRxSession, LatentMeshTxSession, RxConfig, TrustedPeerKeys, TxConfig,
    },
    DroneState, NodeId, Position3D, Velocity3D,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    const SOURCE: u32 = 7;
    const RECEIVER: u32 = 9;
    const NOW_MS: u64 = 10_000;

    let source_key = SigningKey::from_bytes(&[7; 32]);
    let receiver_key = SigningKey::from_bytes(&[9; 32]);

    let mut sender_trust = TrustedPeerKeys::new();
    sender_trust.insert(RECEIVER, receiver_key.verifying_key());
    let mut receiver_trust = TrustedPeerKeys::new();
    receiver_trust.insert(SOURCE, source_key.verifying_key());

    let mut sender = LatentMeshNode::new(
        LatentMeshTxSession::new(
            SOURCE,
            1,
            source_key,
            TxConfig {
                frame_mtu: 64,
                ..TxConfig::default()
            },
        )?,
        LatentMeshRxSession::new(sender_trust, RxConfig::default())?,
        AdaptivePolicy::default(),
    );
    let mut receiver = LatentMeshNode::new(
        LatentMeshTxSession::new(RECEIVER, 1, receiver_key, TxConfig::default())?,
        LatentMeshRxSession::new(receiver_trust, RxConfig::default())?,
        AdaptivePolicy::default(),
    );

    let (left, mut right) = bounded_channel_loopback(16)?;
    let state = DroneState {
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
    };
    let batch = sender.encode_advisory_state(&state, &FailSafeState::Nominal)?;
    for frame in batch.encoded_frames()? {
        left.send_frame(&frame).await?;
    }

    let mut verified = None;
    for _ in 0..batch.frames.len() {
        let frame = right.receive_frame().await?;
        verified = receiver
            .ingest_frame_bytes(SOURCE, NOW_MS, &frame)?
            .or(verified);
    }
    let verified = verified.ok_or("signed advisory did not complete")?;
    let metrics = receiver.metrics().snapshot();
    println!(
        "source={} sequence={} fragments={} logical_bytes={} wire_bytes={} battery={:.1}% authority={:?}",
        verified.received().metadata.source_id,
        verified.received().metadata.logical_sequence,
        verified.received().metadata.fragment_count,
        metrics.received_logical_bytes,
        metrics.received_wire_bytes,
        verified.received().snapshot.drone.battery_pct,
        verified.security_context().requested_authority(),
    );
    Ok(())
}
