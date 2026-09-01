//! Safe composition boundary for protocol, policy, and operational metrics.

use crate::{failsafe::FailSafeState, types::DroneState};

use super::{
    metrics::{DropCounter, LatentMeshMetrics},
    policy::{AdaptivePolicy, DeliveryDecision, LinkSnapshot, MessageIntent, SecurityContext},
    protocol::{
        EnvelopeSigner, LatentMeshRxSession, LatentMeshTxSession, ProtocolError, ProtocolResult,
        ReceivedAdvisorySnapshot, ReplayCheckpoint, TransmitBatch, TrustedPeerKeyLookup,
    },
};

/// A verified observation paired with locally constructed trust metadata.
///
/// The fields are private so a deserialized radio payload cannot manufacture
/// this type. It remains advisory and carries no flight-control capability.
#[derive(Clone, Debug)]
pub struct VerifiedAdvisory {
    received: ReceivedAdvisorySnapshot,
    security: SecurityContext,
}

impl VerifiedAdvisory {
    pub fn received(&self) -> &ReceivedAdvisorySnapshot {
        &self.received
    }

    pub fn security_context(&self) -> &SecurityContext {
        &self.security
    }

    pub fn into_received(self) -> ReceivedAdvisorySnapshot {
        self.received
    }
}

/// Per-vehicle LatentMesh endpoint.
///
/// This facade intentionally owns no `FlightController`, `MeshTopology`,
/// geofence, actuator, or fail-safe mutation handle. It can publish read-only
/// local telemetry, verify remote observations, and evaluate delivery policy.
pub struct LatentMeshNode<S, K> {
    transmitter: LatentMeshTxSession<S>,
    receiver: LatentMeshRxSession<K>,
    policy: AdaptivePolicy,
    metrics: LatentMeshMetrics,
}

impl<S: EnvelopeSigner, K: TrustedPeerKeyLookup> LatentMeshNode<S, K> {
    pub fn new(
        transmitter: LatentMeshTxSession<S>,
        receiver: LatentMeshRxSession<K>,
        policy: AdaptivePolicy,
    ) -> Self {
        Self {
            transmitter,
            receiver,
            policy,
            metrics: LatentMeshMetrics::new(),
        }
    }

    /// Encode local state as authenticated advisory telemetry.
    pub fn encode_advisory_state(
        &mut self,
        state: &DroneState,
        failsafe: &FailSafeState,
    ) -> ProtocolResult<TransmitBatch> {
        let batch = self.transmitter.encode_advisory_state(state, failsafe)?;
        let logical_bytes = batch.envelope.encoded_len() as u64;
        let wire_bytes = batch.frames.iter().fold(0_u64, |total, frame| {
            total.saturating_add(frame.encoded_len() as u64)
        });
        self.metrics.record_send(logical_bytes, wire_bytes);
        Ok(batch)
    }

    /// Terminate one untrusted Air frame at the authenticated advisory seam.
    ///
    /// `expected_source_id` must come from a locally configured transport-peer
    /// mapping. The protocol binds it to the signed envelope before admission.
    pub fn ingest_frame_bytes(
        &mut self,
        expected_source_id: u32,
        received_at_ms: u64,
        bytes: &[u8],
    ) -> ProtocolResult<Option<VerifiedAdvisory>> {
        match self
            .receiver
            .ingest_frame_bytes(expected_source_id, received_at_ms, bytes)
        {
            Ok(Some(received)) => {
                let logical_bytes = received.metadata.encoded_message_bytes as u64;
                let wire_bytes = logical_bytes.saturating_add(
                    u64::from(received.metadata.fragment_count)
                        .saturating_mul(latentmesh_air_core::FRAME_MIN_BYTES as u64),
                );
                self.metrics.record_receive(logical_bytes, wire_bytes);
                self.metrics
                    .record_reassembly_completed(u64::from(received.metadata.fragment_count));

                let context_age = self
                    .receiver
                    .config()
                    .advisory_ttl_ms
                    .min(self.policy.config().capability.max_context_age_ms);
                Ok(Some(VerifiedAdvisory {
                    received,
                    security: SecurityContext::verified_observe_only(
                        received_at_ms,
                        received_at_ms.saturating_add(context_age),
                    ),
                }))
            }
            Ok(None) => Ok(None),
            Err(error) => {
                self.record_protocol_drop(&error);
                Err(error)
            }
        }
    }

    /// Evaluate an outbound or locally interpreted message without increasing
    /// its authority. Rejections are counted without retaining payload data.
    pub fn evaluate_delivery(
        &self,
        now_ms: u64,
        link: &LinkSnapshot,
        intent: &MessageIntent,
    ) -> DeliveryDecision {
        let decision = self.policy.evaluate(now_ms, link, intent);
        if let DeliveryDecision::Drop(reason) = decision {
            self.metrics.record_policy_drop(reason);
        }
        decision
    }

    pub fn approve_manifest(&mut self, manifest_id: u64) -> bool {
        self.policy.approve_manifest(manifest_id)
    }

    pub fn revoke_manifest(&mut self, manifest_id: u64) -> bool {
        self.policy.revoke_manifest(manifest_id)
    }

    pub fn expire_stale(&mut self, now_ms: u64) -> usize {
        self.receiver.expire_stale(now_ms)
    }

    pub fn replay_checkpoints(&self) -> Vec<ReplayCheckpoint> {
        self.receiver.replay_checkpoints()
    }

    pub fn restore_replay_checkpoint(
        &mut self,
        checkpoint: ReplayCheckpoint,
    ) -> ProtocolResult<()> {
        self.receiver.restore_replay_checkpoint(checkpoint)
    }

    pub fn receiver(&self) -> &LatentMeshRxSession<K> {
        &self.receiver
    }

    pub fn policy(&self) -> &AdaptivePolicy {
        &self.policy
    }

    pub fn metrics(&self) -> &LatentMeshMetrics {
        &self.metrics
    }

    fn record_protocol_drop(&self, error: &ProtocolError) {
        let counter = match error {
            ProtocolError::AuthenticationFailed
            | ProtocolError::SignatureRequired
            | ProtocolError::UntrustedPeer(_)
            | ProtocolError::SourceHintMismatch { .. } => DropCounter::Authentication,
            ProtocolError::Replay { .. }
            | ProtocolError::TooOld { .. }
            | ProtocolError::StaleEpoch { .. }
            | ProtocolError::StaleLogical { .. }
            | ProtocolError::CheckpointRollback { .. } => DropCounter::Replay,
            ProtocolError::StaleTimestamp { .. }
            | ProtocolError::SnapshotTooOld { .. }
            | ProtocolError::SnapshotFromFuture { .. }
            | ProtocolError::ReceiveClockRegression { .. }
            | ProtocolError::KeyframeRequired { .. } => DropCounter::Stale,
            _ => DropCounter::Policy,
        };
        self.metrics.record_drop(counter);
        if matches!(error, ProtocolError::LearnedResidualForbidden) {
            self.metrics.record_residual_suppressed();
        }
        if matches!(error, ProtocolError::Air(_)) {
            self.metrics.record_reassembly_failure();
        }
    }
}

#[cfg(test)]
mod tests {
    use ed25519_dalek::{Signer, SigningKey};
    use latentmesh_air_core::{
        fragment_message, state_hash_tag, CriticalState, FragmentMeta, FrameFlags, Residual,
        SemanticClass, SemanticDelta, SemanticEnvelope, WireProfile,
    };

    use super::*;
    use crate::{
        latentmesh::{
            policy::{AckMode, AuthorityLevel, RequestedAction, TrafficClass},
            protocol::{stream_id_for_source, RxConfig, TrustedPeerKeys, TxConfig},
            state::to_critical_state,
        },
        types::{NodeId, Position3D, Velocity3D},
    };

    const NOW: u64 = 10_000;

    fn drone(source_id: u32) -> DroneState {
        DroneState {
            id: NodeId(source_id),
            position: Position3D {
                x: 12.25,
                y: -8.5,
                z: -30.0,
            },
            velocity: Velocity3D {
                vx: 2.0,
                vy: -0.5,
                vz: 0.0,
            },
            heading_rad: 1.5,
            altitude_agl_m: 30.0,
            battery_pct: 80.0,
            link_quality: 0.9,
            timestamp_ms: NOW,
        }
    }

    fn node(
        receiver_key: ed25519_dalek::VerifyingKey,
    ) -> LatentMeshNode<SigningKey, TrustedPeerKeys> {
        let source_id = 7;
        let signing_key = SigningKey::from_bytes(&[7; 32]);
        let transmitter = LatentMeshTxSession::new(
            source_id,
            1,
            signing_key,
            TxConfig {
                frame_mtu: 64,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let mut keys = TrustedPeerKeys::new();
        keys.insert(source_id, receiver_key);
        let receiver = LatentMeshRxSession::new(keys, RxConfig::default()).unwrap();
        LatentMeshNode::new(transmitter, receiver, AdaptivePolicy::default())
    }

    fn link() -> LinkSnapshot {
        LinkSnapshot {
            observed_at_ms: NOW,
            rtt_ms: 20.0,
            packet_loss: 0.05,
            throughput_bps: 1_000_000.0,
            queue_delay_ms: 2.0,
            energy_per_byte_mj: 0.001,
            energy_remaining_mj: 100_000.0,
        }
    }

    #[test]
    fn verified_roundtrip_creates_local_observe_only_context() {
        let signing_key = SigningKey::from_bytes(&[7; 32]);
        let mut node = node(signing_key.verifying_key());
        let batch = node
            .encode_advisory_state(&drone(7), &FailSafeState::Nominal)
            .unwrap();
        let mut verified = None;
        for frame in batch.encoded_frames().unwrap() {
            verified = node
                .ingest_frame_bytes(7, NOW, &frame)
                .unwrap()
                .or(verified);
        }
        let verified = verified.expect("complete signed advisory");
        assert!(verified.security_context().is_authenticated());
        assert!(verified.security_context().is_trusted_source());
        assert_eq!(
            verified.security_context().requested_authority(),
            AuthorityLevel::ObserveOnly
        );
        assert_eq!(verified.received().snapshot.drone.id, NodeId(7));

        let intent = MessageIntent {
            traffic_class: TrafficClass::StateSync,
            logical_bytes: 256,
            wire_bytes: 192,
            utility: 0.9,
            created_at_ms: NOW,
            deadline_at_ms: NOW + 1_000,
            encode_energy_mj: 0.2,
            security: *verified.security_context(),
            action: RequestedAction::StateSync,
            residual: None,
        };
        let plan = node
            .evaluate_delivery(NOW, &link(), &intent)
            .plan()
            .expect("fresh verified state should be schedulable");
        assert_eq!(plan.ack, AckMode::Opportunistic);
        assert_eq!(node.metrics().snapshot().received_messages, 1);
    }

    #[test]
    fn wrong_key_never_creates_verified_context() {
        let mut node = node(SigningKey::from_bytes(&[9; 32]).verifying_key());
        let batch = node
            .encode_advisory_state(&drone(7), &FailSafeState::Nominal)
            .unwrap();
        let frames = batch.encoded_frames().unwrap();
        let mut error = None;
        for frame in frames {
            match node.ingest_frame_bytes(7, NOW, &frame) {
                Ok(None) => {}
                Ok(Some(_)) => panic!("wrong key admitted advisory"),
                Err(found) => error = Some(found),
            }
        }
        assert_eq!(error, Some(ProtocolError::AuthenticationFailed));
        assert_eq!(node.metrics().snapshot().authentication_drops, 1);
    }

    #[test]
    fn verified_observation_still_cannot_request_flight_action() {
        let signing_key = SigningKey::from_bytes(&[7; 32]);
        let mut node = node(signing_key.verifying_key());
        let batch = node
            .encode_advisory_state(&drone(7), &FailSafeState::Nominal)
            .unwrap();
        let mut verified = None;
        for frame in batch.encoded_frames().unwrap() {
            verified = node
                .ingest_frame_bytes(7, NOW, &frame)
                .unwrap()
                .or(verified);
        }
        let verified = verified.unwrap();
        let intent = MessageIntent {
            traffic_class: TrafficClass::CriticalCoordination,
            logical_bytes: 64,
            wire_bytes: 64,
            utility: 1.0,
            created_at_ms: NOW,
            deadline_at_ms: NOW + 1_000,
            encode_energy_mj: 0.1,
            security: *verified.security_context(),
            action: RequestedAction::Arm,
            residual: None,
        };
        assert_eq!(
            node.evaluate_delivery(NOW, &link(), &intent),
            DeliveryDecision::Drop(super::super::policy::PolicyDropReason::ForbiddenAction)
        );
    }

    #[test]
    fn residual_envelope_is_rejected_and_counted_without_snapshot() {
        let source_id = 7;
        let signing_key = SigningKey::from_bytes(&[7; 32]);
        let mut endpoint = node(signing_key.verifying_key());
        let target = to_critical_state(&drone(source_id), &FailSafeState::Nominal).unwrap();
        let residuals = (0..latentmesh_air_core::MAX_RESIDUALS)
            .map(|slot| Residual {
                slot: u8::try_from(slot).unwrap(),
                importance: u8::MAX,
                scale_exp: 0,
                values: vec![i8::MAX; latentmesh_air_core::MAX_RESIDUAL_VALUES],
            })
            .collect();
        let delta =
            SemanticDelta::between(source_id, 1, 0, &CriticalState::new(), &target, residuals)
                .unwrap();
        let mut envelope = SemanticEnvelope::wrap_delta(&delta, 12, 0, Some([0; 64])).unwrap();
        envelope.signature = Some(
            signing_key
                .sign(&envelope.authentication_bytes().unwrap())
                .to_bytes(),
        );
        let frames = fragment_message(
            FragmentMeta {
                profile: WireProfile::Wifi,
                flags: FrameFlags::SIGNED_ENVELOPE,
                stream_id: stream_id_for_source(source_id),
                sequence: 0,
                class: SemanticClass::StateDelta,
                priority: 12,
                state_tag: state_hash_tag(&delta.result_hash),
            },
            &envelope.encode().unwrap(),
            256,
        )
        .unwrap();

        let mut found = None;
        for frame in frames {
            if let Err(error) =
                endpoint.ingest_frame_bytes(source_id, NOW, &frame.encode().unwrap())
            {
                found = Some(error);
            }
        }
        assert_eq!(found, Some(ProtocolError::LearnedResidualForbidden));
        let metrics = endpoint.metrics().snapshot();
        assert_eq!(metrics.residual_suppressions, 1);
        assert_eq!(metrics.received_messages, 0);
        assert!(endpoint.receiver().peer_snapshot(source_id).is_none());
        assert!(endpoint.replay_checkpoints().is_empty());
    }
}
