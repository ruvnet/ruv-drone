//! LatentMesh authenticated advisory communications and governed orchestration.
//!
//! This feature is deliberately outside the flight-control authority path.
//! Decoded peer snapshots are observations only and must never be wired into
//! actuator commands, geofence or failsafe overrides, or collision avoidance.

pub mod metrics;
pub mod node;
pub mod policy;
pub mod protocol;
pub mod state;
pub mod transport;

pub use metrics::{DropCounter, LatentMeshMetrics, MetricsSnapshot};
pub use node::{LatentMeshNode, VerifiedAdvisory};
pub use policy::{
    AckMode, AdaptivePolicy, AdaptivePolicyConfig, AuthorityLevel, AuthorizedCapability,
    CapabilityGate, CapabilityGateConfig, DeliveryDecision, DeliveryPlan, LinkSnapshot,
    MessageIntent, PolicyDropReason, Redundancy, RequestedAction, ResidualEvidence,
    ScheduledDecision, SecurityContext, TrafficClass, MAX_AUTHORITY,
};
pub use protocol::{
    stream_id_for_source, EnvelopeSigner, LatentMeshRxSession, LatentMeshTxSession, ProtocolError,
    ProtocolMetrics, ProtocolResult, ReceiveMetadata, ReceivedAdvisorySnapshot, ReplayCheckpoint,
    RxConfig, TransmitBatch, TrustedPeerKeyLookup, TrustedPeerKeys, TxConfig,
};
pub use state::{
    from_critical_state, to_critical_state, AdvisoryPeerSnapshot, StateError,
    CRITICAL_STATE_SCHEMA_VERSION,
};
pub use transport::{
    bounded_channel_loopback, ChannelFrameTransport, FrameTransport, TransportError,
    TransportResult, UdpFrameTransport,
};
