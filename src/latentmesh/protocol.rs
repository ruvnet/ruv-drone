//! Authenticated LatentMesh advisory telemetry sessions for cooperative drones.
//!
//! This module terminates the untrusted radio protocol. A successful receive
//! result is still advisory data: it is not authority for flight control,
//! collision avoidance, geofencing, failsafe transitions, or topology state.

use std::collections::BTreeMap;

use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use latentmesh_air_core::{
    fragment_message, state_hash_tag, AirError, CriticalState, FragmentMeta, FrameFlags,
    ReassembledMessage, Reassembler, ReassemblerConfig, ReplayDecision, ReplayWindow,
    SemanticClass, SemanticDelta, SemanticEnvelope, SparseRadioFrame, WireProfile, FRAME_MAX_BYTES,
    FRAME_MIN_BYTES,
};

use crate::{failsafe::FailSafeState, types::DroneState};

use super::state::{from_critical_state, to_critical_state, AdvisoryPeerSnapshot, StateError};

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ProtocolError {
    #[error("LatentMesh Air rejected the message: {0}")]
    Air(#[from] AirError),
    #[error("advisory state validation failed: {0}")]
    State(#[from] StateError),
    #[error("source {0} has no trusted Ed25519 public key")]
    UntrustedPeer(u32),
    #[error("Ed25519 envelope authentication failed")]
    AuthenticationFailed,
    #[error("signed envelope is required")]
    SignatureRequired,
    #[error("outer frame metadata does not match signed field {0}")]
    MetadataMismatch(&'static str),
    #[error("source {source_id} does not match advisory node {node_id}")]
    SourceMismatch { source_id: u32, node_id: u32 },
    #[error("replayed message from source {source_id}, epoch {epoch}")]
    Replay { source_id: u32, epoch: u32 },
    #[error("message is outside the replay window for source {source_id}, epoch {epoch}")]
    TooOld { source_id: u32, epoch: u32 },
    #[error("epoch {received} is stale for source {source_id}; current epoch is {current}")]
    StaleEpoch {
        source_id: u32,
        received: u32,
        current: u32,
    },
    #[error(
        "logical sequence {received} is not newer for source {source_id}; last accepted is {last}"
    )]
    StaleLogical {
        source_id: u32,
        received: u64,
        last: u64,
    },
    #[error("epoch {epoch} for source {source_id} must start at logical sequence zero")]
    InvalidEpochStart { source_id: u32, epoch: u32 },
    #[error(
        "advisory timestamp {received} precedes accepted timestamp {last} for source {source_id}"
    )]
    StaleTimestamp {
        source_id: u32,
        received: u64,
        last: u64,
    },
    #[error("learned residuals are forbidden in deterministic drone telemetry")]
    LearnedResidualForbidden,
    #[error("configured peer bound of {0} is exhausted")]
    PeerCapacity(usize),
    #[error("the current epoch exhausted its monotonic logical sequence")]
    SequenceExhausted,
    #[error("signing provider could not sign the envelope")]
    SigningFailed,
    #[error("authenticated source {actual} does not match transport peer hint {expected}")]
    SourceHintMismatch { expected: u32, actual: u32 },
    #[error("pre-authentication frame rate exceeded for source hint {source_id}")]
    RateLimited { source_id: u32 },
    #[error("receive clock regressed for source hint {source_id}")]
    ReceiveClockRegression { source_id: u32 },
    #[error("snapshot from source {source_id} is older than the configured receive window")]
    SnapshotTooOld { source_id: u32 },
    #[error("snapshot from source {source_id} exceeds configured future clock skew")]
    SnapshotFromFuture { source_id: u32 },
    #[error("source {source_id}, epoch {epoch} requires a signed full keyframe")]
    KeyframeRequired { source_id: u32, epoch: u32 },
    #[error("replay checkpoint would roll source {source_id} high-water backwards")]
    CheckpointRollback { source_id: u32 },
    #[error("new epoch {new_epoch} must be greater than current epoch {current_epoch}")]
    InvalidEpochRotation { current_epoch: u32, new_epoch: u32 },
    #[error("receive state for source hint {source_id} was unavailable after admission")]
    IngressUnavailable { source_id: u32 },
}

pub type ProtocolResult<T> = Result<T, ProtocolError>;

/// Synchronous signing seam for software keys, OS keystores, or HSM adapters.
/// Implementations must produce an Ed25519 signature over the exact bytes.
pub trait EnvelopeSigner {
    fn sign_envelope(&self, authentication_bytes: &[u8]) -> ProtocolResult<[u8; 64]>;
}

impl EnvelopeSigner for SigningKey {
    fn sign_envelope(&self, authentication_bytes: &[u8]) -> ProtocolResult<[u8; 64]> {
        Ok(self.sign(authentication_bytes).to_bytes())
    }
}

/// Trusted-key lookup is intentionally keyed by the signed `source_id`, not by
/// transport address or an unauthenticated outer frame field.
pub trait TrustedPeerKeyLookup {
    fn key_for_source(&self, source_id: u32) -> Option<VerifyingKey>;
}

#[derive(Clone, Debug, Default)]
pub struct TrustedPeerKeys {
    keys: BTreeMap<u32, VerifyingKey>,
}

impl TrustedPeerKeys {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, source_id: u32, key: VerifyingKey) -> Option<VerifyingKey> {
        self.keys.insert(source_id, key)
    }

    pub fn remove(&mut self, source_id: u32) -> Option<VerifyingKey> {
        self.keys.remove(&source_id)
    }

    pub fn len(&self) -> usize {
        self.keys.len()
    }

    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }
}

impl TrustedPeerKeyLookup for TrustedPeerKeys {
    fn key_for_source(&self, source_id: u32) -> Option<VerifyingKey> {
        self.keys.get(&source_id).cloned()
    }
}

impl TrustedPeerKeyLookup for BTreeMap<u32, VerifyingKey> {
    fn key_for_source(&self, source_id: u32) -> Option<VerifyingKey> {
        self.get(&source_id).cloned()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TxConfig {
    pub profile: WireProfile,
    /// Complete SparseRadioFrame MTU, including its 16 byte overhead.
    pub frame_mtu: usize,
    pub priority: u8,
    /// Emit a complete signed state keyframe at this message interval.
    /// This bounds recovery after an unacknowledged datagram loss.
    pub keyframe_interval: u32,
    /// Transport flags such as FEC. SIGNED_ENVELOPE is added by the session.
    pub transport_flags: FrameFlags,
}

impl Default for TxConfig {
    fn default() -> Self {
        Self {
            profile: WireProfile::Wifi,
            frame_mtu: FRAME_MAX_BYTES,
            priority: 12,
            keyframe_interval: 16,
            transport_flags: FrameFlags::NONE,
        }
    }
}

impl TxConfig {
    fn validate(self) -> ProtocolResult<()> {
        if !(FRAME_MIN_BYTES..=FRAME_MAX_BYTES).contains(&self.frame_mtu) {
            return Err(ProtocolError::Air(AirError::InvalidLength));
        }
        if self.priority > 15 || self.keyframe_interval == 0 {
            return Err(ProtocolError::Air(AirError::InvalidLength));
        }
        FrameFlags::from_bits(self.transport_flags.bits())?;
        if self.transport_flags.contains(FrameFlags::SIGNED_ENVELOPE) {
            return Err(ProtocolError::MetadataMismatch("transport_flags"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct TransmitBatch {
    pub source_id: u32,
    pub epoch: u32,
    pub message_id: u32,
    pub logical_sequence: u64,
    pub wire_sequence: u16,
    pub state_hash: [u8; 16],
    pub keyframe: bool,
    pub envelope: SemanticEnvelope,
    pub frames: Vec<SparseRadioFrame>,
}

impl TransmitBatch {
    pub fn encoded_frames(&self) -> ProtocolResult<Vec<Vec<u8>>> {
        self.frames
            .iter()
            .map(SparseRadioFrame::encode)
            .collect::<Result<Vec<_>, _>>()
            .map_err(ProtocolError::from)
    }
}

/// Deterministic signed-transmit session. State and counters are committed only
/// after the complete envelope has been encoded and fragmented successfully.
pub struct LatentMeshTxSession<S = SigningKey> {
    source_id: u32,
    epoch: u32,
    next_logical_sequence: u64,
    last_state: CriticalState,
    signer: S,
    config: TxConfig,
}

impl<S: EnvelopeSigner> LatentMeshTxSession<S> {
    pub fn new(source_id: u32, epoch: u32, signer: S, config: TxConfig) -> ProtocolResult<Self> {
        config.validate()?;
        Ok(Self {
            source_id,
            epoch,
            next_logical_sequence: 0,
            last_state: CriticalState::new(),
            signer,
            config,
        })
    }

    pub const fn source_id(&self) -> u32 {
        self.source_id
    }

    pub const fn epoch(&self) -> u32 {
        self.epoch
    }

    pub const fn next_logical_sequence(&self) -> u64 {
        self.next_logical_sequence
    }

    pub fn rotate_epoch(&mut self, new_epoch: u32) -> ProtocolResult<()> {
        if new_epoch <= self.epoch {
            return Err(ProtocolError::InvalidEpochRotation {
                current_epoch: self.epoch,
                new_epoch,
            });
        }
        self.epoch = new_epoch;
        self.next_logical_sequence = 0;
        self.last_state = CriticalState::new();
        Ok(())
    }

    pub fn encode_advisory_state(
        &mut self,
        drone: &DroneState,
        failsafe: &FailSafeState,
    ) -> ProtocolResult<TransmitBatch> {
        if drone.id.0 != self.source_id {
            return Err(ProtocolError::SourceMismatch {
                source_id: self.source_id,
                node_id: drone.id.0,
            });
        }
        let logical_sequence = self.next_logical_sequence;
        let message_id =
            u32::try_from(logical_sequence).map_err(|_| ProtocolError::SequenceExhausted)?;
        let next_logical_sequence = logical_sequence
            .checked_add(1)
            .ok_or(ProtocolError::SequenceExhausted)?;
        let target = to_critical_state(drone, failsafe)?;
        let keyframe = logical_sequence.is_multiple_of(u64::from(self.config.keyframe_interval));
        let empty = CriticalState::new();
        let base = if keyframe { &empty } else { &self.last_state };
        let delta = SemanticDelta::between(
            self.source_id,
            self.epoch,
            message_id,
            base,
            &target,
            Vec::new(),
        )?;

        let mut envelope = SemanticEnvelope::wrap_delta(
            &delta,
            self.config.priority,
            logical_sequence,
            Some([0_u8; 64]),
        )?;
        let authentication_bytes = envelope.authentication_bytes()?;
        envelope.signature = Some(self.signer.sign_envelope(&authentication_bytes)?);
        let encoded_envelope = envelope.encode()?;
        let wire_sequence = logical_sequence as u16;
        let flags = self
            .config
            .transport_flags
            .union(FrameFlags::SIGNED_ENVELOPE);
        let frames = fragment_message(
            FragmentMeta {
                profile: self.config.profile,
                flags,
                stream_id: stream_id_for_source(self.source_id),
                sequence: wire_sequence,
                class: SemanticClass::StateDelta,
                priority: self.config.priority,
                state_tag: state_hash_tag(&delta.result_hash),
            },
            &encoded_envelope,
            self.config.frame_mtu,
        )?;

        self.last_state = target;
        self.next_logical_sequence = next_logical_sequence;
        Ok(TransmitBatch {
            source_id: self.source_id,
            epoch: self.epoch,
            message_id,
            logical_sequence,
            wire_sequence,
            state_hash: delta.result_hash,
            keyframe,
            envelope,
            frames,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RxConfig {
    pub profile: WireProfile,
    pub transport_flags: FrameFlags,
    pub reassembly: ReassemblerConfig,
    pub max_peers: usize,
    pub max_frames_per_window: u32,
    pub frame_window_ms: u64,
    pub reassembly_timeout_ms: u64,
    pub max_snapshot_age_ms: u64,
    pub max_future_skew_ms: u64,
    pub advisory_ttl_ms: u64,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            profile: WireProfile::Wifi,
            transport_flags: FrameFlags::NONE,
            reassembly: ReassemblerConfig {
                max_contexts: 4,
                max_message_bytes: 4_096,
                max_fragments: latentmesh_air_core::MAX_FRAGMENTS,
            },
            max_peers: 32,
            max_frames_per_window: 128,
            frame_window_ms: 1_000,
            reassembly_timeout_ms: 2_000,
            max_snapshot_age_ms: 5_000,
            max_future_skew_ms: 500,
            advisory_ttl_ms: 5_000,
        }
    }
}

impl RxConfig {
    fn validate(self) -> ProtocolResult<()> {
        if self.max_peers == 0
            || self.max_peers > 256
            || self.max_frames_per_window == 0
            || self.max_frames_per_window > 65_536
            || self.frame_window_ms == 0
            || self.reassembly_timeout_ms == 0
            || self.max_snapshot_age_ms == 0
            || self.advisory_ttl_ms == 0
        {
            return Err(ProtocolError::Air(AirError::InvalidLength));
        }
        FrameFlags::from_bits(self.transport_flags.bits())?;
        if self.transport_flags.contains(FrameFlags::SIGNED_ENVELOPE) {
            return Err(ProtocolError::MetadataMismatch("transport_flags"));
        }
        let _ = Reassembler::new(self.reassembly)?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ProtocolMetrics {
    pub frames_seen: u64,
    pub completed_envelopes: u64,
    pub accepted_snapshots: u64,
    pub crc_rejections: u64,
    pub authentication_rejections: u64,
    pub replay_rejections: u64,
    pub semantic_rejections: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReceiveMetadata {
    pub source_id: u32,
    pub epoch: u32,
    pub message_id: u32,
    pub logical_sequence: u64,
    pub wire_sequence: u16,
    pub stream_id: u16,
    pub priority: u8,
    pub state_hash: [u8; 16],
    /// Complete signed semantic envelope size before Air frame overhead.
    pub encoded_message_bytes: usize,
    /// Number of Air fragments that carried this logical message.
    pub fragment_count: u8,
    /// True when the signed delta is independent of prior receiver state.
    pub keyframe: bool,
    /// Always zero. Nonzero learned residuals are rejected before admission.
    pub learned_residuals: usize,
}

/// Authenticated, deterministic, but non-authoritative advisory telemetry.
#[derive(Clone, Debug)]
pub struct ReceivedAdvisorySnapshot {
    pub snapshot: AdvisoryPeerSnapshot,
    pub metadata: ReceiveMetadata,
}

#[derive(Clone, Debug)]
struct PeerRuntime {
    epoch: u32,
    replay: ReplayWindow,
    last_logical_sequence: u64,
    last_timestamp_ms: u64,
    accepted_at_ms: u64,
    critical: Option<CriticalState>,
    snapshot: Option<AdvisoryPeerSnapshot>,
    requires_keyframe: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ReplayCheckpoint {
    pub source_id: u32,
    pub epoch: u32,
    pub last_logical_sequence: u64,
    pub last_timestamp_ms: u64,
}

#[derive(Clone, Debug)]
struct PeerIngress {
    reassembler: Reassembler,
    context_started_ms: BTreeMap<(u16, u16), u64>,
    last_received_ms: Option<u64>,
    window_started_ms: u64,
    frames_in_window: u32,
}

impl PeerIngress {
    fn new(config: ReassemblerConfig, now_ms: u64) -> ProtocolResult<Self> {
        Ok(Self {
            reassembler: Reassembler::new(config)?,
            context_started_ms: BTreeMap::new(),
            last_received_ms: None,
            window_started_ms: now_ms,
            frames_in_window: 0,
        })
    }
}

/// Receive session with bounded fragment contexts and one replay window per
/// authenticated source and current epoch.
pub struct LatentMeshRxSession<K> {
    keys: K,
    config: RxConfig,
    ingress: BTreeMap<u32, PeerIngress>,
    peers: BTreeMap<u32, PeerRuntime>,
    metrics: ProtocolMetrics,
}

impl<K: TrustedPeerKeyLookup> LatentMeshRxSession<K> {
    pub fn new(keys: K, config: RxConfig) -> ProtocolResult<Self> {
        config.validate()?;
        Ok(Self {
            keys,
            config,
            ingress: BTreeMap::new(),
            peers: BTreeMap::new(),
            metrics: ProtocolMetrics::default(),
        })
    }

    pub const fn metrics(&self) -> ProtocolMetrics {
        self.metrics
    }

    pub const fn config(&self) -> &RxConfig {
        &self.config
    }

    /// Returns advisory data only. Callers must not insert this into the
    /// authoritative topology or use it for direct control decisions.
    pub fn peer_snapshot(&self, source_id: u32) -> Option<&AdvisoryPeerSnapshot> {
        self.peers
            .get(&source_id)
            .and_then(|peer| peer.snapshot.as_ref())
    }

    pub fn replay_checkpoints(&self) -> Vec<ReplayCheckpoint> {
        self.peers
            .iter()
            .map(|(&source_id, peer)| ReplayCheckpoint {
                source_id,
                epoch: peer.epoch,
                last_logical_sequence: peer.last_logical_sequence,
                last_timestamp_ms: peer.last_timestamp_ms,
            })
            .collect()
    }

    /// Restore only replay high-water state. No advisory state is restored, so
    /// a newer signed full keyframe is mandatory before delta admission.
    pub fn restore_replay_checkpoint(
        &mut self,
        checkpoint: ReplayCheckpoint,
    ) -> ProtocolResult<()> {
        if self.keys.key_for_source(checkpoint.source_id).is_none() {
            return Err(ProtocolError::UntrustedPeer(checkpoint.source_id));
        }
        if u32::try_from(checkpoint.last_logical_sequence).is_err() {
            return Err(ProtocolError::SequenceExhausted);
        }
        if let Some(current) = self.peers.get(&checkpoint.source_id) {
            if checkpoint.epoch < current.epoch
                || (checkpoint.epoch == current.epoch
                    && checkpoint.last_logical_sequence <= current.last_logical_sequence)
            {
                return Err(ProtocolError::CheckpointRollback {
                    source_id: checkpoint.source_id,
                });
            }
        } else if self.active_source_count() == self.config.max_peers {
            return Err(ProtocolError::PeerCapacity(self.config.max_peers));
        }
        self.peers.insert(
            checkpoint.source_id,
            PeerRuntime {
                epoch: checkpoint.epoch,
                replay: ReplayWindow::new(),
                last_logical_sequence: checkpoint.last_logical_sequence,
                last_timestamp_ms: checkpoint.last_timestamp_ms,
                accepted_at_ms: 0,
                critical: None,
                snapshot: None,
                requires_keyframe: true,
            },
        );
        Ok(())
    }

    /// Expire advisory snapshots while retaining replay high-water marks.
    pub fn expire_stale(&mut self, now_ms: u64) -> usize {
        let mut expired = 0;
        for peer in self.peers.values_mut() {
            if peer.snapshot.is_some()
                && now_ms.saturating_sub(peer.accepted_at_ms) > self.config.advisory_ttl_ms
            {
                peer.snapshot = None;
                peer.critical = None;
                peer.requires_keyframe = true;
                expired += 1;
            }
        }
        expired
    }

    fn active_source_count(&self) -> usize {
        self.peers.len()
            + self
                .ingress
                .keys()
                .filter(|source_id| !self.peers.contains_key(source_id))
                .count()
    }

    pub fn ingest_frame_bytes(
        &mut self,
        expected_source_id: u32,
        received_at_ms: u64,
        bytes: &[u8],
    ) -> ProtocolResult<Option<ReceivedAdvisorySnapshot>> {
        self.metrics.frames_seen = self.metrics.frames_seen.saturating_add(1);
        let result = SparseRadioFrame::decode(bytes)
            .map_err(ProtocolError::from)
            .and_then(|frame| self.ingest_decoded_frame(expected_source_id, received_at_ms, frame));
        self.record_result(&result);
        result
    }

    pub fn ingest_frame(
        &mut self,
        expected_source_id: u32,
        received_at_ms: u64,
        frame: SparseRadioFrame,
    ) -> ProtocolResult<Option<ReceivedAdvisorySnapshot>> {
        self.metrics.frames_seen = self.metrics.frames_seen.saturating_add(1);
        let result = self.ingest_decoded_frame(expected_source_id, received_at_ms, frame);
        self.record_result(&result);
        result
    }

    fn ingest_decoded_frame(
        &mut self,
        expected_source_id: u32,
        received_at_ms: u64,
        frame: SparseRadioFrame,
    ) -> ProtocolResult<Option<ReceivedAdvisorySnapshot>> {
        if frame.profile != self.config.profile {
            return Err(ProtocolError::MetadataMismatch("profile"));
        }
        let expected_flags = self
            .config
            .transport_flags
            .union(FrameFlags::SIGNED_ENVELOPE);
        if frame.flags.bits() != expected_flags.bits() {
            return Err(ProtocolError::MetadataMismatch("flags"));
        }
        if frame.stream_id != stream_id_for_source(expected_source_id) {
            return Err(ProtocolError::MetadataMismatch("source_hint_stream_id"));
        }
        if self.keys.key_for_source(expected_source_id).is_none() {
            return Err(ProtocolError::UntrustedPeer(expected_source_id));
        }
        if !self.ingress.contains_key(&expected_source_id) {
            if !self.peers.contains_key(&expected_source_id)
                && self.active_source_count() == self.config.max_peers
            {
                return Err(ProtocolError::PeerCapacity(self.config.max_peers));
            }
            self.ingress.insert(
                expected_source_id,
                PeerIngress::new(self.config.reassembly, received_at_ms)?,
            );
        }
        let ingress =
            self.ingress
                .get_mut(&expected_source_id)
                .ok_or(ProtocolError::IngressUnavailable {
                    source_id: expected_source_id,
                })?;
        if ingress
            .last_received_ms
            .is_some_and(|last| received_at_ms < last)
        {
            return Err(ProtocolError::ReceiveClockRegression {
                source_id: expected_source_id,
            });
        }
        let expired_contexts: Vec<(u16, u16)> = ingress
            .context_started_ms
            .iter()
            .filter_map(|(&key, &started)| {
                (received_at_ms.saturating_sub(started) > self.config.reassembly_timeout_ms)
                    .then_some(key)
            })
            .collect();
        for (stream_id, sequence) in expired_contexts {
            ingress.reassembler.clear(stream_id, sequence);
            ingress.context_started_ms.remove(&(stream_id, sequence));
        }
        if received_at_ms.saturating_sub(ingress.window_started_ms) >= self.config.frame_window_ms {
            ingress.window_started_ms = received_at_ms;
            ingress.frames_in_window = 0;
        }
        if ingress.frames_in_window >= self.config.max_frames_per_window {
            return Err(ProtocolError::RateLimited {
                source_id: expected_source_id,
            });
        }
        ingress.frames_in_window += 1;
        ingress.last_received_ms = Some(received_at_ms);
        let fragment_count = frame.fragment_count;
        let context_key = (frame.stream_id, frame.sequence);
        let is_new_context = !ingress
            .reassembler
            .has_in_flight(frame.stream_id, frame.sequence);
        if is_new_context && ingress.context_started_ms.len() == self.config.reassembly.max_contexts
        {
            return Err(ProtocolError::Air(AirError::ReassemblyFull));
        }
        if is_new_context {
            ingress
                .context_started_ms
                .insert(context_key, received_at_ms);
        }
        let complete = match ingress.reassembler.push(frame) {
            Ok(complete) => complete,
            Err(error) => {
                if is_new_context {
                    ingress.context_started_ms.remove(&context_key);
                }
                return Err(error.into());
            }
        };
        let Some(message) = complete else {
            return Ok(None);
        };
        ingress.context_started_ms.remove(&context_key);
        self.metrics.completed_envelopes = self.metrics.completed_envelopes.saturating_add(1);
        self.admit_complete(expected_source_id, received_at_ms, fragment_count, message)
            .map(Some)
    }

    fn admit_complete(
        &mut self,
        expected_source_id: u32,
        received_at_ms: u64,
        fragment_count: u8,
        message: ReassembledMessage,
    ) -> ProtocolResult<ReceivedAdvisorySnapshot> {
        let envelope = SemanticEnvelope::decode(&message.bytes)?;
        if envelope.source_id != expected_source_id {
            return Err(ProtocolError::SourceHintMismatch {
                expected: expected_source_id,
                actual: envelope.source_id,
            });
        }
        if envelope.class != message.class {
            return Err(ProtocolError::MetadataMismatch("class"));
        }
        if envelope.priority != message.priority {
            return Err(ProtocolError::MetadataMismatch("priority"));
        }
        if state_hash_tag(&envelope.state_hash) != message.state_tag {
            return Err(ProtocolError::MetadataMismatch("state_tag"));
        }
        if message.stream_id != stream_id_for_source(envelope.source_id) {
            return Err(ProtocolError::MetadataMismatch("stream_id"));
        }
        if message.sequence != envelope.logical_sequence as u16 {
            return Err(ProtocolError::MetadataMismatch("sequence"));
        }
        let expected_message_id = u32::try_from(envelope.logical_sequence)
            .map_err(|_| ProtocolError::SequenceExhausted)?;
        if envelope.message_id != expected_message_id {
            return Err(ProtocolError::MetadataMismatch("message_id"));
        }
        let signature_bytes = envelope
            .signature
            .as_ref()
            .ok_or(ProtocolError::SignatureRequired)?;
        let key = self
            .keys
            .key_for_source(envelope.source_id)
            .ok_or(ProtocolError::UntrustedPeer(envelope.source_id))?;
        let signature = Signature::from_bytes(signature_bytes);
        let authentication_bytes = envelope.authentication_bytes()?;
        key.verify_strict(&authentication_bytes, &signature)
            .map_err(|_| ProtocolError::AuthenticationFailed)?;

        let delta = envelope.unwrap_delta()?;
        if !delta.residuals.is_empty() {
            return Err(ProtocolError::LearnedResidualForbidden);
        }
        let existing = self.peers.get(&envelope.source_id);
        let empty = CriticalState::new();
        let keyframe = delta.base_hash == empty.critical_hash();
        let (base, mut replay) = match existing {
            Some(peer) if envelope.epoch < peer.epoch => {
                return Err(ProtocolError::StaleEpoch {
                    source_id: envelope.source_id,
                    received: envelope.epoch,
                    current: peer.epoch,
                });
            }
            Some(peer) if envelope.epoch == peer.epoch => {
                if peer.requires_keyframe && !keyframe {
                    return Err(ProtocolError::KeyframeRequired {
                        source_id: envelope.source_id,
                        epoch: envelope.epoch,
                    });
                }
                let base = if keyframe {
                    empty.clone()
                } else {
                    peer.critical
                        .clone()
                        .ok_or(ProtocolError::KeyframeRequired {
                            source_id: envelope.source_id,
                            epoch: envelope.epoch,
                        })?
                };
                (base, peer.replay)
            }
            Some(_) | None => {
                if envelope.logical_sequence != 0 || !keyframe {
                    return Err(ProtocolError::InvalidEpochStart {
                        source_id: envelope.source_id,
                        epoch: envelope.epoch,
                    });
                }
                (empty, ReplayWindow::new())
            }
        };

        match replay.classify(message.sequence) {
            ReplayDecision::Accept => {}
            ReplayDecision::Duplicate => {
                return Err(ProtocolError::Replay {
                    source_id: envelope.source_id,
                    epoch: envelope.epoch,
                });
            }
            ReplayDecision::TooOld => {
                return Err(ProtocolError::TooOld {
                    source_id: envelope.source_id,
                    epoch: envelope.epoch,
                });
            }
        }

        if let Some(peer) = existing {
            if envelope.epoch == peer.epoch
                && envelope.logical_sequence <= peer.last_logical_sequence
            {
                return Err(ProtocolError::StaleLogical {
                    source_id: envelope.source_id,
                    received: envelope.logical_sequence,
                    last: peer.last_logical_sequence,
                });
            }
        }

        let result = delta.apply(&base)?;
        let snapshot = from_critical_state(&result)?;
        if snapshot.drone.id.0 != envelope.source_id {
            return Err(ProtocolError::SourceMismatch {
                source_id: envelope.source_id,
                node_id: snapshot.drone.id.0,
            });
        }
        if snapshot.drone.timestamp_ms
            > received_at_ms.saturating_add(self.config.max_future_skew_ms)
        {
            return Err(ProtocolError::SnapshotFromFuture {
                source_id: envelope.source_id,
            });
        }
        if received_at_ms.saturating_sub(snapshot.drone.timestamp_ms)
            > self.config.max_snapshot_age_ms
        {
            return Err(ProtocolError::SnapshotTooOld {
                source_id: envelope.source_id,
            });
        }
        if let Some(peer) = existing {
            if envelope.epoch == peer.epoch && snapshot.drone.timestamp_ms < peer.last_timestamp_ms
            {
                return Err(ProtocolError::StaleTimestamp {
                    source_id: envelope.source_id,
                    received: snapshot.drone.timestamp_ms,
                    last: peer.last_timestamp_ms,
                });
            }
        }

        // This is the only replay commit. It happens after full reassembly,
        // CRC checks, metadata binding, signature verification, hash-based
        // delta application, and advisory schema validation all succeed.
        replay.commit(message.sequence)?;
        if existing.is_none() && self.active_source_count() > self.config.max_peers {
            return Err(ProtocolError::PeerCapacity(self.config.max_peers));
        }
        let metadata = ReceiveMetadata {
            source_id: envelope.source_id,
            epoch: envelope.epoch,
            message_id: envelope.message_id,
            logical_sequence: envelope.logical_sequence,
            wire_sequence: message.sequence,
            stream_id: message.stream_id,
            priority: envelope.priority,
            state_hash: delta.result_hash,
            encoded_message_bytes: message.bytes.len(),
            fragment_count,
            keyframe,
            learned_residuals: 0,
        };
        self.peers.insert(
            envelope.source_id,
            PeerRuntime {
                epoch: envelope.epoch,
                replay,
                last_logical_sequence: envelope.logical_sequence,
                last_timestamp_ms: snapshot.drone.timestamp_ms,
                accepted_at_ms: received_at_ms,
                critical: Some(result),
                snapshot: Some(snapshot.clone()),
                requires_keyframe: false,
            },
        );
        self.metrics.accepted_snapshots = self.metrics.accepted_snapshots.saturating_add(1);
        Ok(ReceivedAdvisorySnapshot { snapshot, metadata })
    }

    fn record_result(&mut self, result: &ProtocolResult<Option<ReceivedAdvisorySnapshot>>) {
        let Err(error) = result else {
            return;
        };
        match error {
            ProtocolError::Air(AirError::CrcMismatch) => {
                self.metrics.crc_rejections = self.metrics.crc_rejections.saturating_add(1);
            }
            ProtocolError::AuthenticationFailed
            | ProtocolError::SignatureRequired
            | ProtocolError::UntrustedPeer(_) => {
                self.metrics.authentication_rejections =
                    self.metrics.authentication_rejections.saturating_add(1);
            }
            ProtocolError::Replay { .. }
            | ProtocolError::TooOld { .. }
            | ProtocolError::StaleEpoch { .. }
            | ProtocolError::StaleLogical { .. } => {
                self.metrics.replay_rejections = self.metrics.replay_rejections.saturating_add(1);
            }
            _ => {
                self.metrics.semantic_rejections =
                    self.metrics.semantic_rejections.saturating_add(1);
            }
        }
    }
}

/// Stable, deterministic stream routing. Security does not depend on the
/// collision-prone 16 bit value; source identity comes from the signed envelope.
pub const fn stream_id_for_source(source_id: u32) -> u16 {
    (source_id as u16) ^ ((source_id >> 16) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{NodeId, Position3D, Velocity3D};

    fn signing_key(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }

    fn sample(source_id: u32) -> DroneState {
        DroneState {
            id: NodeId(source_id),
            position: Position3D {
                x: 12.25,
                y: -8.5,
                z: -31.0,
            },
            velocity: Velocity3D {
                vx: 2.0,
                vy: -0.5,
                vz: 0.25,
            },
            heading_rad: 1.5,
            altitude_agl_m: 31.0,
            battery_pct: 81.25,
            link_quality: 0.9375,
            timestamp_ms: 100,
        }
    }

    fn receiver(source_id: u32, key: VerifyingKey) -> LatentMeshRxSession<TrustedPeerKeys> {
        let mut keys = TrustedPeerKeys::new();
        keys.insert(source_id, key);
        LatentMeshRxSession::new(keys, RxConfig::default()).unwrap()
    }

    fn receiver_for(
        peers: &[(u32, VerifyingKey)],
        config: RxConfig,
    ) -> LatentMeshRxSession<TrustedPeerKeys> {
        let mut keys = TrustedPeerKeys::new();
        for (source_id, key) in peers {
            keys.insert(*source_id, *key);
        }
        LatentMeshRxSession::new(keys, config).unwrap()
    }

    fn deliver(
        rx: &mut LatentMeshRxSession<TrustedPeerKeys>,
        expected_source_id: u32,
        received_at_ms: u64,
        frames: &[SparseRadioFrame],
    ) -> ProtocolResult<ReceivedAdvisorySnapshot> {
        let mut result = None;
        for frame in frames {
            if let Some(received) =
                rx.ingest_frame_bytes(expected_source_id, received_at_ms, &frame.encode().unwrap())?
            {
                result = Some(received);
            }
        }
        Ok(result.expect("complete envelope"))
    }

    fn reframe(envelope: &SemanticEnvelope, mtu: usize) -> Vec<SparseRadioFrame> {
        fragment_message(
            FragmentMeta {
                profile: WireProfile::Wifi,
                flags: FrameFlags::SIGNED_ENVELOPE,
                stream_id: stream_id_for_source(envelope.source_id),
                sequence: envelope.logical_sequence as u16,
                class: envelope.class,
                priority: envelope.priority,
                state_tag: state_hash_tag(&envelope.state_hash),
            },
            &envelope.encode().unwrap(),
            mtu,
        )
        .unwrap()
    }

    #[test]
    fn signed_state_round_trip_reassembles_out_of_order() {
        let source_id = 42;
        let key = signing_key(7);
        for (profile, frame_mtu) in [
            (WireProfile::Wifi, 64),
            (WireProfile::Ble, 64),
            (WireProfile::Meshtastic, 227),
        ] {
            let mut tx = LatentMeshTxSession::new(
                source_id,
                9,
                key.clone(),
                TxConfig {
                    profile,
                    frame_mtu,
                    ..TxConfig::default()
                },
            )
            .unwrap();
            let batch = tx
                .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
                .unwrap();
            assert!(batch.frames.len() > 1);

            let mut rx = receiver_for(
                &[(source_id, key.verifying_key())],
                RxConfig {
                    profile,
                    ..RxConfig::default()
                },
            );
            let duplicate = batch.frames[0].encode().unwrap();
            assert!(rx
                .ingest_frame_bytes(source_id, 100, &duplicate)
                .unwrap()
                .is_none());
            assert!(rx
                .ingest_frame_bytes(source_id, 100, &duplicate)
                .unwrap()
                .is_none());
            let mut received = None;
            for frame in batch
                .frames
                .iter()
                .rev()
                .filter(|frame| frame.fragment_index != 0)
            {
                if let Some(update) = rx
                    .ingest_frame_bytes(source_id, 100, &frame.encode().unwrap())
                    .unwrap()
                {
                    received = Some(update);
                }
            }
            let received = received.unwrap();
            assert_eq!(received.snapshot.drone.id, NodeId(source_id));
            assert_eq!(received.snapshot.failsafe, FailSafeState::Nominal);
            assert_eq!(received.metadata.epoch, 9);
            assert!(received.metadata.keyframe);
            assert_eq!(received.metadata.learned_residuals, 0);
            assert_eq!(rx.metrics().accepted_snapshots, 1);
        }
    }

    #[test]
    fn tampered_signature_is_rejected_without_consuming_replay_slot() {
        let source_id = 7;
        let key = signing_key(2);
        let mut tx = LatentMeshTxSession::new(
            source_id,
            1,
            key.clone(),
            TxConfig {
                frame_mtu: 64,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut tampered = batch.envelope.clone();
        tampered.signature.as_mut().unwrap()[0] ^= 0x80;

        let mut rx = receiver(source_id, key.verifying_key());
        let error = deliver(&mut rx, source_id, 100, &reframe(&tampered, 64)).unwrap_err();
        assert_eq!(error, ProtocolError::AuthenticationFailed);
        let accepted = deliver(&mut rx, source_id, 100, &batch.frames).unwrap();
        assert_eq!(accepted.metadata.logical_sequence, 0);
    }

    #[test]
    fn wrong_trusted_key_is_rejected() {
        let source_id = 9;
        let key = signing_key(3);
        let mut tx = LatentMeshTxSession::new(source_id, 1, key, TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut rx = receiver(source_id, signing_key(4).verifying_key());
        assert_eq!(
            deliver(&mut rx, source_id, 100, &batch.frames).unwrap_err(),
            ProtocolError::AuthenticationFailed
        );
    }

    #[test]
    fn unsigned_source_hint_and_profile_mismatches_fail_before_admission() {
        let source_id = 31;
        let colliding_hint = 0x0001_001e;
        assert_eq!(
            stream_id_for_source(source_id),
            stream_id_for_source(colliding_hint)
        );
        let key = signing_key(31);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();

        let mut unsigned = batch.envelope.clone();
        unsigned.signature = None;
        let mut rx = receiver(source_id, key.verifying_key());
        assert_eq!(
            deliver(&mut rx, source_id, 100, &reframe(&unsigned, 64)).unwrap_err(),
            ProtocolError::SignatureRequired
        );
        assert!(rx.peer_snapshot(source_id).is_none());

        let mut source_bound = receiver_for(
            &[
                (source_id, key.verifying_key()),
                (colliding_hint, key.verifying_key()),
            ],
            RxConfig::default(),
        );
        assert!(matches!(
            deliver(&mut source_bound, colliding_hint, 100, &batch.frames),
            Err(ProtocolError::SourceHintMismatch { expected, actual })
                if expected == colliding_hint && actual == source_id
        ));
        assert!(source_bound.peer_snapshot(source_id).is_none());

        let mut wrong_profile = receiver_for(
            &[(source_id, key.verifying_key())],
            RxConfig {
                profile: WireProfile::Ble,
                ..RxConfig::default()
            },
        );
        assert_eq!(
            wrong_profile
                .ingest_frame(source_id, 100, batch.frames[0].clone())
                .unwrap_err(),
            ProtocolError::MetadataMismatch("profile")
        );
        assert!(wrong_profile.peer_snapshot(source_id).is_none());
    }

    #[test]
    fn replay_of_fully_verified_envelope_is_rejected() {
        let source_id = 11;
        let key = signing_key(5);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut rx = receiver(source_id, key.verifying_key());
        deliver(&mut rx, source_id, 100, &batch.frames).unwrap();
        assert!(matches!(
            deliver(&mut rx, source_id, 100, &batch.frames),
            Err(ProtocolError::Replay {
                source_id: 11,
                epoch: 1,
            })
        ));
        assert_eq!(rx.metrics().replay_rejections, 1);
    }

    #[test]
    fn signed_outer_metadata_tamper_is_rejected_after_valid_crc() {
        let source_id = 13;
        let key = signing_key(6);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut tampered = batch.frames.clone();
        for frame in &mut tampered {
            frame.state_tag ^= 1;
        }
        let mut rx = receiver(source_id, key.verifying_key());
        assert_eq!(
            deliver(&mut rx, source_id, 100, &tampered).unwrap_err(),
            ProtocolError::MetadataMismatch("state_tag")
        );
    }

    #[test]
    fn signed_control_class_is_rejected_without_state_admission() {
        let source_id = 14;
        let key = signing_key(14);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut control = batch.envelope;
        control.class = SemanticClass::Control;
        let authentication_bytes = control.authentication_bytes().unwrap();
        control.signature = Some(key.sign(&authentication_bytes).to_bytes());

        let mut rx = receiver(source_id, key.verifying_key());
        assert_eq!(
            deliver(&mut rx, source_id, 100, &reframe(&control, 64)).unwrap_err(),
            ProtocolError::Air(AirError::InvalidClass)
        );
        assert!(rx.peer_snapshot(source_id).is_none());
        assert!(rx.replay_checkpoints().is_empty());
    }

    #[test]
    fn oversized_untrusted_datagram_is_rejected_before_allocation() {
        let source_id = 15;
        let key = signing_key(8);
        let mut rx = receiver(source_id, key.verifying_key());
        let error = rx.ingest_frame_bytes(source_id, 100, &vec![0_u8; FRAME_MAX_BYTES + 1]);
        assert!(matches!(
            error,
            Err(ProtocolError::Air(AirError::InvalidLength))
        ));
    }

    #[test]
    fn hostile_datagram_lengths_and_fragment_bounds_fail_closed() {
        let source_id = 151;
        let key = signing_key(51);
        let mut rx = receiver(source_id, key.verifying_key());
        for length in 0..=8_192 {
            assert!(rx
                .ingest_frame_bytes(source_id, 100, &vec![0_u8; length])
                .is_err());
        }

        let fragment = |sequence| SparseRadioFrame {
            profile: WireProfile::Wifi,
            flags: FrameFlags::SIGNED_ENVELOPE,
            stream_id: stream_id_for_source(source_id),
            sequence,
            fragment_index: 0,
            fragment_count: 2,
            class: SemanticClass::StateDelta,
            priority: 12,
            state_tag: 0,
            payload: vec![0],
        };
        for sequence in 0..4 {
            assert!(rx
                .ingest_frame_bytes(source_id, 101, &fragment(sequence).encode().unwrap())
                .unwrap()
                .is_none());
        }
        let mut conflict = fragment(0);
        conflict.payload[0] = 1;
        assert!(matches!(
            rx.ingest_frame_bytes(source_id, 101, &conflict.encode().unwrap()),
            Err(ProtocolError::Air(AirError::FragmentConflict))
        ));
        assert!(matches!(
            rx.ingest_frame_bytes(source_id, 101, &fragment(4).encode().unwrap()),
            Err(ProtocolError::Air(AirError::ReassemblyFull))
        ));

        for fragment_count in 0..=u8::MAX {
            let mut candidate = fragment(5);
            candidate.fragment_count = fragment_count;
            if (1..=latentmesh_air_core::MAX_FRAGMENTS).contains(&fragment_count) {
                assert!(candidate.encode().is_ok());
            } else {
                assert_eq!(candidate.encode(), Err(AirError::InvalidFragment));
            }
        }
    }

    #[test]
    fn periodic_keyframe_recovers_after_dropped_delta_chain() {
        let source_id = 16;
        let key = signing_key(10);
        let mut tx = LatentMeshTxSession::new(
            source_id,
            1,
            key.clone(),
            TxConfig {
                keyframe_interval: 3,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let mut rx = receiver(source_id, key.verifying_key());

        let mut state = sample(source_id);
        let initial = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(initial.keyframe);
        deliver(&mut rx, source_id, 100, &initial.frames).unwrap();

        state.position.x = 13.0;
        state.timestamp_ms = 101;
        let dropped = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(!dropped.keyframe);

        state.position.x = 14.0;
        state.timestamp_ms = 102;
        let broken_chain = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(!broken_chain.keyframe);
        assert!(matches!(
            deliver(&mut rx, source_id, 102, &broken_chain.frames),
            Err(ProtocolError::Air(AirError::BaseStateMismatch))
        ));

        state.position.x = 15.0;
        state.timestamp_ms = 103;
        let recovery = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(recovery.keyframe);
        let received = deliver(&mut rx, source_id, 103, &recovery.frames).unwrap();
        assert!(received.metadata.keyframe);
        assert!((received.snapshot.drone.position.x - 15.0).abs() < 0.000_02);
        assert_eq!(received.metadata.logical_sequence, 3);
    }

    #[test]
    fn zero_keyframe_interval_is_rejected() {
        let result = LatentMeshTxSession::new(
            1,
            1,
            signing_key(11),
            TxConfig {
                keyframe_interval: 0,
                ..TxConfig::default()
            },
        );
        assert!(matches!(
            result,
            Err(ProtocolError::Air(AirError::InvalidLength))
        ));
    }

    #[test]
    fn colliding_stream_ids_are_isolated_by_authenticated_peer_hint() {
        let source_a = 1;
        let source_b = 0x0001_0000;
        assert_eq!(
            stream_id_for_source(source_a),
            stream_id_for_source(source_b)
        );
        let key_a = signing_key(21);
        let key_b = signing_key(22);
        let config = TxConfig {
            frame_mtu: 64,
            ..TxConfig::default()
        };
        let mut tx_a = LatentMeshTxSession::new(source_a, 1, key_a.clone(), config).unwrap();
        let mut tx_b = LatentMeshTxSession::new(source_b, 1, key_b.clone(), config).unwrap();
        let batch_a = tx_a
            .encode_advisory_state(&sample(source_a), &FailSafeState::Nominal)
            .unwrap();
        let batch_b = tx_b
            .encode_advisory_state(&sample(source_b), &FailSafeState::AutonomousHold)
            .unwrap();
        let mut rx = receiver_for(
            &[
                (source_a, key_a.verifying_key()),
                (source_b, key_b.verifying_key()),
            ],
            RxConfig::default(),
        );

        let mut received = Vec::new();
        for index in 0..batch_a.frames.len().max(batch_b.frames.len()) {
            if let Some(frame) = batch_a.frames.get(index) {
                if let Some(snapshot) = rx
                    .ingest_frame_bytes(source_a, 100, &frame.encode().unwrap())
                    .unwrap()
                {
                    received.push(snapshot.snapshot.drone.id.0);
                }
            }
            if let Some(frame) = batch_b.frames.get(index) {
                if let Some(snapshot) = rx
                    .ingest_frame_bytes(source_b, 100, &frame.encode().unwrap())
                    .unwrap()
                {
                    received.push(snapshot.snapshot.drone.id.0);
                }
            }
        }
        received.sort_unstable();
        assert_eq!(received, vec![source_a, source_b]);
    }

    #[test]
    fn preauthentication_frame_rate_is_bounded_per_peer_hint() {
        let source_id = 23;
        let key = signing_key(23);
        let mut tx = LatentMeshTxSession::new(
            source_id,
            1,
            key.clone(),
            TxConfig {
                frame_mtu: 64,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        assert!(batch.frames.len() > 2);
        let mut rx = receiver_for(
            &[(source_id, key.verifying_key())],
            RxConfig {
                max_frames_per_window: 2,
                ..RxConfig::default()
            },
        );
        assert!(rx
            .ingest_frame_bytes(source_id, 100, &batch.frames[0].encode().unwrap())
            .unwrap()
            .is_none());
        assert!(rx
            .ingest_frame_bytes(source_id, 100, &batch.frames[1].encode().unwrap())
            .unwrap()
            .is_none());
        assert!(matches!(
            rx.ingest_frame_bytes(source_id, 100, &batch.frames[2].encode().unwrap()),
            Err(ProtocolError::RateLimited { source_id: 23 })
        ));
    }

    #[test]
    fn partial_reassembly_has_a_hard_first_fragment_deadline() {
        let source_id = 24;
        let key = signing_key(24);
        let mut tx = LatentMeshTxSession::new(
            source_id,
            1,
            key.clone(),
            TxConfig {
                frame_mtu: 64,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut rx = receiver_for(
            &[(source_id, key.verifying_key())],
            RxConfig {
                reassembly_timeout_ms: 10,
                ..RxConfig::default()
            },
        );
        assert!(rx
            .ingest_frame_bytes(source_id, 100, &batch.frames[0].encode().unwrap())
            .unwrap()
            .is_none());
        for frame in &batch.frames[1..] {
            assert!(rx
                .ingest_frame_bytes(source_id, 111, &frame.encode().unwrap())
                .unwrap()
                .is_none());
        }
        let recovered = rx
            .ingest_frame_bytes(source_id, 112, &batch.frames[0].encode().unwrap())
            .unwrap()
            .unwrap();
        assert_eq!(recovered.snapshot.drone.id.0, source_id);
    }

    #[test]
    fn signed_snapshot_freshness_rejects_stale_and_future_clocks() {
        let source_id = 25;
        let key = signing_key(25);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let config = RxConfig {
            max_snapshot_age_ms: 10,
            max_future_skew_ms: 5,
            ..RxConfig::default()
        };
        let mut stale_rx = receiver_for(&[(source_id, key.verifying_key())], config);
        assert!(matches!(
            deliver(&mut stale_rx, source_id, 111, &batch.frames),
            Err(ProtocolError::SnapshotTooOld { source_id: 25 })
        ));
        let mut future_rx = receiver_for(&[(source_id, key.verifying_key())], config);
        assert!(matches!(
            deliver(&mut future_rx, source_id, 94, &batch.frames),
            Err(ProtocolError::SnapshotFromFuture { source_id: 25 })
        ));
    }

    #[test]
    fn advisory_snapshot_ttl_expires_data_but_keeps_replay_checkpoint() {
        let source_id = 26;
        let key = signing_key(26);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let batch = tx
            .encode_advisory_state(&sample(source_id), &FailSafeState::Nominal)
            .unwrap();
        let mut rx = receiver_for(
            &[(source_id, key.verifying_key())],
            RxConfig {
                advisory_ttl_ms: 10,
                ..RxConfig::default()
            },
        );
        deliver(&mut rx, source_id, 100, &batch.frames).unwrap();
        assert!(rx.peer_snapshot(source_id).is_some());
        assert_eq!(rx.expire_stale(110), 0);
        assert_eq!(rx.expire_stale(111), 1);
        assert!(rx.peer_snapshot(source_id).is_none());
        assert_eq!(rx.replay_checkpoints().len(), 1);
    }

    #[test]
    fn restored_replay_highwater_rejects_old_messages_until_next_keyframe() {
        let source_id = 27;
        let key = signing_key(27);
        let mut tx = LatentMeshTxSession::new(
            source_id,
            1,
            key.clone(),
            TxConfig {
                keyframe_interval: 3,
                ..TxConfig::default()
            },
        )
        .unwrap();
        let mut state = sample(source_id);
        let initial = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        let mut first_rx = receiver(source_id, key.verifying_key());
        deliver(&mut first_rx, source_id, 100, &initial.frames).unwrap();
        let checkpoint = first_rx.replay_checkpoints()[0];

        state.timestamp_ms = 101;
        let non_keyframe = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        state.timestamp_ms = 102;
        let _dropped = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        state.timestamp_ms = 103;
        state.position.x = 27.0;
        let keyframe = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(keyframe.keyframe);

        let mut restored = receiver(source_id, key.verifying_key());
        restored.restore_replay_checkpoint(checkpoint).unwrap();
        assert!(matches!(
            deliver(&mut restored, source_id, 100, &initial.frames),
            Err(ProtocolError::StaleLogical {
                source_id: 27,
                received: 0,
                last: 0
            })
        ));
        assert!(matches!(
            deliver(&mut restored, source_id, 101, &non_keyframe.frames),
            Err(ProtocolError::KeyframeRequired {
                source_id: 27,
                epoch: 1
            })
        ));
        let recovered = deliver(&mut restored, source_id, 103, &keyframe.frames).unwrap();
        assert!((recovered.snapshot.drone.position.x - 27.0).abs() < 0.000_02);
    }

    #[test]
    fn replay_checkpoint_requires_an_enrolled_source_and_reachable_sequence() {
        let source_id = 28;
        let mut untrusted = receiver_for(&[], RxConfig::default());
        let checkpoint = ReplayCheckpoint {
            source_id,
            epoch: 1,
            last_logical_sequence: 0,
            last_timestamp_ms: 100,
        };
        assert_eq!(
            untrusted.restore_replay_checkpoint(checkpoint),
            Err(ProtocolError::UntrustedPeer(source_id))
        );

        let key = signing_key(28);
        let mut trusted = receiver(source_id, key.verifying_key());
        assert_eq!(
            trusted.restore_replay_checkpoint(ReplayCheckpoint {
                last_logical_sequence: u64::from(u32::MAX) + 1,
                ..checkpoint
            }),
            Err(ProtocolError::SequenceExhausted)
        );
        assert!(trusted.replay_checkpoints().is_empty());
    }

    #[test]
    fn logical_sequence_remains_monotonic_across_wire_sequence_wrap() {
        let source_id = 29;
        let key = signing_key(29);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let mut rx = receiver(source_id, key.verifying_key());
        let mut state = sample(source_id);
        let initial = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        deliver(&mut rx, source_id, 100, &initial.frames).unwrap();

        let peer = rx.peers.get_mut(&source_id).unwrap();
        peer.replay = ReplayWindow::new();
        peer.replay.commit(u16::MAX).unwrap();
        peer.last_logical_sequence = u64::from(u16::MAX);
        tx.next_logical_sequence = u64::from(u16::MAX) + 1;
        state.timestamp_ms = 101;
        state.position.x = 29.0;

        let wrapped = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert_eq!(wrapped.wire_sequence, 0);
        let received = deliver(&mut rx, source_id, 101, &wrapped.frames).unwrap();
        assert_eq!(received.metadata.logical_sequence, u64::from(u16::MAX) + 1);
        assert_eq!(received.metadata.wire_sequence, 0);
    }

    #[test]
    fn epoch_rotation_requires_a_greater_epoch_starting_with_a_keyframe() {
        let source_id = 30;
        let key = signing_key(30);
        let mut tx =
            LatentMeshTxSession::new(source_id, 1, key.clone(), TxConfig::default()).unwrap();
        let mut rx = receiver(source_id, key.verifying_key());
        let mut state = sample(source_id);
        let old = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        deliver(&mut rx, source_id, 100, &old.frames).unwrap();

        tx.rotate_epoch(2).unwrap();
        state.timestamp_ms = 101;
        let rotated = tx
            .encode_advisory_state(&state, &FailSafeState::Nominal)
            .unwrap();
        assert!(rotated.keyframe);
        assert_eq!(rotated.logical_sequence, 0);
        assert_eq!(
            deliver(&mut rx, source_id, 101, &rotated.frames)
                .unwrap()
                .metadata
                .epoch,
            2
        );
        assert!(matches!(
            deliver(&mut rx, source_id, 102, &old.frames),
            Err(ProtocolError::StaleEpoch {
                source_id: 30,
                received: 1,
                current: 2
            })
        ));
        assert_eq!(
            tx.rotate_epoch(2),
            Err(ProtocolError::InvalidEpochRotation {
                current_epoch: 2,
                new_epoch: 2
            })
        );
    }

    #[test]
    fn learned_residual_cannot_enter_advisory_snapshot() {
        let source_id = 17;
        let key = signing_key(9);
        let target = to_critical_state(&sample(source_id), &FailSafeState::Nominal).unwrap();
        let delta = SemanticDelta::between(
            source_id,
            1,
            0,
            &CriticalState::new(),
            &target,
            vec![latentmesh_air_core::Residual {
                slot: 0,
                importance: 255,
                scale_exp: 0,
                values: vec![127],
            }],
        )
        .unwrap();
        let mut envelope = SemanticEnvelope::wrap_delta(&delta, 12, 0, Some([0_u8; 64])).unwrap();
        let auth = envelope.authentication_bytes().unwrap();
        envelope.signature = Some(key.sign(&auth).to_bytes());
        let mut rx = receiver(source_id, key.verifying_key());
        assert_eq!(
            deliver(
                &mut rx,
                source_id,
                100,
                &reframe(&envelope, FRAME_MAX_BYTES)
            )
            .unwrap_err(),
            ProtocolError::LearnedResidualForbidden
        );
    }
}
