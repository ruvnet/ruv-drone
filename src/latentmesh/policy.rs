//! Deterministic admission and delivery policy for the LatentMesh data plane.
//!
//! This module deliberately stops at **mission coordination**.  A message may
//! report state or propose bounded cooperative work, but it can never command
//! an actuator or alter flight authority.  Priority, redundancy, and learned
//! compression are transport choices; none of them can raise authority.

use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BTreeSet;

/// The highest authority any LatentMesh message may request in this crate.
pub const MAX_AUTHORITY: AuthorityLevel = AuthorityLevel::MissionCoordination;

/// Coarse authority tiers, ordered by consequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum AuthorityLevel {
    /// Read or synchronize information without influencing a decision.
    ObserveOnly,
    /// Nonbinding advice or a bounded cooperative task proposal.
    MissionCoordination,
    /// Any command that can change vehicle, actuator, or payload state.
    FlightAuthority,
}

/// Semantic action requested by a received envelope.
///
/// Denied variants are intentionally explicit.  They must remain denied even
/// when a sender is authenticated, a link is degraded, or a residual model has
/// excellent measured utility.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequestedAction {
    Observe,
    StateSync,
    SnapshotRequest,
    NonbindingAdvisory,
    CooperativeTaskProposal {
        /// Opaque reference to a locally approved mission manifest.  The wire
        /// action carries no coordinates, setpoints, or executable payload.
        manifest_id: u64,
        participant_count: u16,
        ttl_ms: u32,
        effort_units: u32,
    },
    ActuatorCommand,
    Arm,
    Disarm,
    FlightModeChange,
    AttitudeSetpoint,
    RateSetpoint,
    VelocitySetpoint,
    PositionSetpoint,
    GeofenceOverride,
    FailsafeOverride,
    PayloadRelease,
    /// Unknown wire discriminants must not inherit a permissive default.
    Unknown(u16),
}

/// The capability emitted after admission.  There is deliberately no flight
/// or actuator capability in this type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizedCapability {
    /// Read-only use.  A learned residual admitted here remains ephemeral and
    /// must never be merged into canonical deterministic flight state.
    Observe,
    StateSync,
    SnapshotRequest,
    NonbindingAdvisory,
    CooperativeTaskProposal,
}

/// Authenticated envelope metadata.  It contains no key material or payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SecurityContext {
    /// Local verifier output.  Deserialization always resets this to false so
    /// an envelope cannot self-assert authentication.
    #[serde(skip_deserializing, default)]
    authenticated: bool,
    /// Local trust-store output.  Deserialization always resets this to false.
    #[serde(skip_deserializing, default)]
    trusted_source: bool,
    issued_at_ms: u64,
    expires_at_ms: u64,
    requested_authority: AuthorityLevel,
}

impl SecurityContext {
    /// Construct the only security context emitted by the current receive
    /// protocol. This is intentionally unavailable outside the crate so a
    /// library consumer cannot mint authenticated authority metadata.
    pub(super) const fn verified_observe_only(issued_at_ms: u64, expires_at_ms: u64) -> Self {
        Self {
            authenticated: true,
            trusted_source: true,
            issued_at_ms,
            expires_at_ms,
            requested_authority: AuthorityLevel::ObserveOnly,
        }
    }

    pub const fn is_authenticated(&self) -> bool {
        self.authenticated
    }

    pub const fn is_trusted_source(&self) -> bool {
        self.trusted_source
    }

    pub const fn issued_at_ms(&self) -> u64 {
        self.issued_at_ms
    }

    pub const fn expires_at_ms(&self) -> u64 {
        self.expires_at_ms
    }

    pub const fn requested_authority(&self) -> AuthorityLevel {
        self.requested_authority
    }
}

/// Hard limits for the capability gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityGateConfig {
    /// Maximum age of an authenticated decision context.  Default: 30 s.
    pub max_context_age_ms: u64,
    /// Accepted positive clock skew.  Default: 2 s.
    pub max_clock_skew_ms: u64,
    /// Cooperative proposals are bounded to this many peers.  Default: 64.
    pub max_proposal_participants: u16,
    /// Proposal TTL ceiling.  Default: 60 s.
    pub max_proposal_ttl_ms: u32,
    /// Abstract, deployment-defined effort ceiling.  Default: 10,000 units.
    pub max_proposal_effort_units: u32,
}

impl Default for CapabilityGateConfig {
    fn default() -> Self {
        Self {
            max_context_age_ms: 30_000,
            max_clock_skew_ms: 2_000,
            max_proposal_participants: 64,
            max_proposal_ttl_ms: 60_000,
            max_proposal_effort_units: 10_000,
        }
    }
}

/// Default-deny action and authority gate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CapabilityGate {
    config: CapabilityGateConfig,
    approved_manifest_ids: BTreeSet<u64>,
}

impl CapabilityGate {
    pub fn new(config: CapabilityGateConfig) -> Self {
        Self {
            config,
            approved_manifest_ids: BTreeSet::new(),
        }
    }

    pub fn config(&self) -> &CapabilityGateConfig {
        &self.config
    }

    /// Add a manifest approved by the local mission authority.  There is no
    /// receive-side API that calls this method; remote messages cannot approve
    /// their own manifest.  Zero is reserved as "no manifest".
    pub fn approve_manifest(&mut self, manifest_id: u64) -> bool {
        manifest_id != 0 && self.approved_manifest_ids.insert(manifest_id)
    }

    pub fn revoke_manifest(&mut self, manifest_id: u64) -> bool {
        self.approved_manifest_ids.remove(&manifest_id)
    }

    pub fn is_manifest_approved(&self, manifest_id: u64) -> bool {
        manifest_id != 0 && self.approved_manifest_ids.contains(&manifest_id)
    }

    /// Admit only a fresh, authenticated, trusted, explicitly allowed action.
    pub fn authorize(
        &self,
        now_ms: u64,
        context: &SecurityContext,
        action: RequestedAction,
    ) -> Result<AuthorizedCapability, PolicyDropReason> {
        if !context.authenticated {
            return Err(PolicyDropReason::Unauthenticated);
        }
        if !context.trusted_source {
            return Err(PolicyDropReason::UntrustedSource);
        }
        if context.requested_authority > MAX_AUTHORITY {
            return Err(PolicyDropReason::ExcessiveAuthority);
        }
        if context.issued_at_ms > now_ms.saturating_add(self.config.max_clock_skew_ms)
            || context.expires_at_ms < context.issued_at_ms
            || now_ms > context.expires_at_ms
            || now_ms.saturating_sub(context.issued_at_ms) > self.config.max_context_age_ms
        {
            return Err(PolicyDropReason::StaleInput);
        }

        let (capability, action_ceiling) = match action {
            RequestedAction::Observe => {
                (AuthorizedCapability::Observe, AuthorityLevel::ObserveOnly)
            }
            RequestedAction::StateSync => {
                (AuthorizedCapability::StateSync, AuthorityLevel::ObserveOnly)
            }
            RequestedAction::SnapshotRequest => (
                AuthorizedCapability::SnapshotRequest,
                AuthorityLevel::ObserveOnly,
            ),
            RequestedAction::NonbindingAdvisory => (
                AuthorizedCapability::NonbindingAdvisory,
                AuthorityLevel::MissionCoordination,
            ),
            RequestedAction::CooperativeTaskProposal {
                manifest_id,
                participant_count,
                ttl_ms,
                effort_units,
            } => {
                if !self.is_manifest_approved(manifest_id) {
                    return Err(PolicyDropReason::ManifestNotApproved);
                }
                if participant_count == 0
                    || participant_count > self.config.max_proposal_participants
                    || ttl_ms == 0
                    || ttl_ms > self.config.max_proposal_ttl_ms
                    || effort_units == 0
                    || effort_units > self.config.max_proposal_effort_units
                {
                    return Err(PolicyDropReason::ProposalOutOfBounds);
                }
                (
                    AuthorizedCapability::CooperativeTaskProposal,
                    AuthorityLevel::MissionCoordination,
                )
            }
            RequestedAction::Unknown(_) => return Err(PolicyDropReason::UnknownAction),
            RequestedAction::ActuatorCommand
            | RequestedAction::Arm
            | RequestedAction::Disarm
            | RequestedAction::FlightModeChange
            | RequestedAction::AttitudeSetpoint
            | RequestedAction::RateSetpoint
            | RequestedAction::VelocitySetpoint
            | RequestedAction::PositionSetpoint
            | RequestedAction::GeofenceOverride
            | RequestedAction::FailsafeOverride
            | RequestedAction::PayloadRelease => return Err(PolicyDropReason::ForbiddenAction),
        };

        // An observe-only action cannot smuggle a broader authority request.
        if context.requested_authority > action_ceiling {
            return Err(PolicyDropReason::ExcessiveAuthority);
        }
        Ok(capability)
    }
}

impl Default for CapabilityGate {
    fn default() -> Self {
        Self::new(CapabilityGateConfig::default())
    }
}

/// Traffic classes used for deterministic scheduling.  They describe delivery
/// treatment, not execution authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TrafficClass {
    CriticalCoordination,
    CooperativeTask,
    StateSync,
    SnapshotRequest,
    Observation,
    LearnedResidual,
    BulkSnapshot,
}

impl TrafficClass {
    fn priority_weight(self) -> f64 {
        match self {
            Self::CriticalCoordination => 8.0,
            Self::CooperativeTask => 6.0,
            Self::StateSync => 4.0,
            Self::SnapshotRequest => 3.0,
            Self::Observation => 2.0,
            Self::LearnedResidual => 1.5,
            Self::BulkSnapshot => 1.0,
        }
    }

    /// Freshness ceilings are transport defaults, not flight-control timing:
    /// 2 s for coordination/residuals, 3 s for state, 5 s for observations,
    /// 10 s for requests, and 60 s for a bulk snapshot.
    pub fn max_age_ms(self) -> u64 {
        match self {
            Self::CriticalCoordination | Self::CooperativeTask | Self::LearnedResidual => 2_000,
            Self::StateSync => 3_000,
            Self::Observation => 5_000,
            Self::SnapshotRequest => 10_000,
            Self::BulkSnapshot => 60_000,
        }
    }

    fn permits(self, action: RequestedAction) -> bool {
        matches!(
            (self, action),
            (
                Self::CriticalCoordination,
                RequestedAction::NonbindingAdvisory
            ) | (
                Self::CooperativeTask,
                RequestedAction::CooperativeTaskProposal { .. }
            ) | (Self::StateSync, RequestedAction::StateSync)
                | (Self::SnapshotRequest, RequestedAction::SnapshotRequest)
                | (Self::Observation, RequestedAction::Observe)
                | (Self::LearnedResidual, RequestedAction::Observe)
                | (Self::BulkSnapshot, RequestedAction::Observe)
        )
    }
}

/// Local, measured link state.  Every floating-point field is validated before
/// use so NaN cannot silently disable a threshold comparison.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkSnapshot {
    pub observed_at_ms: u64,
    pub rtt_ms: f64,
    /// Pre-FEC packet loss in `[0, 1]`.
    pub packet_loss: f64,
    pub throughput_bps: f64,
    pub queue_delay_ms: f64,
    pub energy_per_byte_mj: f64,
    pub energy_remaining_mj: f64,
}

impl LinkSnapshot {
    fn valid(&self) -> bool {
        self.rtt_ms.is_finite()
            && self.rtt_ms >= 0.0
            && self.packet_loss.is_finite()
            && (0.0..=1.0).contains(&self.packet_loss)
            && self.throughput_bps.is_finite()
            && self.throughput_bps > 0.0
            && self.queue_delay_ms.is_finite()
            && self.queue_delay_ms >= 0.0
            && self.energy_per_byte_mj.is_finite()
            && self.energy_per_byte_mj >= 0.0
            && self.energy_remaining_mj.is_finite()
            && self.energy_remaining_mj >= 0.0
    }
}

/// Evidence required before an opaque learned residual can use the link.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResidualEvidence {
    pub evaluated_at_ms: u64,
    pub causal_controls_passed: bool,
    pub sample_count: u32,
    pub measured_delta_utility: f64,
    pub p_value: f64,
    pub heldout_relative_residual: f64,
    pub task_success_delta: f64,
    pub model_compatible: bool,
    pub decoder_trusted: bool,
    pub semantic_fallback_available: bool,
}

impl ResidualEvidence {
    fn valid_metrics(&self) -> bool {
        self.measured_delta_utility.is_finite()
            && self.p_value.is_finite()
            && self.heldout_relative_residual.is_finite()
            && self.task_success_delta.is_finite()
            && (-1.0..=1.0).contains(&self.measured_delta_utility)
            && (0.0..=1.0).contains(&self.p_value)
            && self.heldout_relative_residual >= 0.0
            && (-1.0..=1.0).contains(&self.task_success_delta)
    }
}

/// One candidate message before link policy is applied.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MessageIntent {
    pub traffic_class: TrafficClass,
    pub logical_bytes: u32,
    /// Encoded wire bytes before adaptive FEC or duplicate copies.
    pub wire_bytes: u32,
    /// Measured or application-supplied utility in `[0, 1]`.
    pub utility: f64,
    pub created_at_ms: u64,
    pub deadline_at_ms: u64,
    /// Encoding/compute energy not represented by per-byte radio energy.
    pub encode_energy_mj: f64,
    pub security: SecurityContext,
    pub action: RequestedAction,
    pub residual: Option<ResidualEvidence>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AckMode {
    None,
    Opportunistic,
    Required,
}

/// Transport redundancy selected from measured loss.  `copies` includes the
/// original transmission; parity is additional FEC overhead per copy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Redundancy {
    pub copies: u8,
    pub parity_percent: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeliveryPlan {
    pub capability: AuthorizedCapability,
    pub ack: AckMode,
    pub redundancy: Redundancy,
    pub estimated_wire_bytes: u64,
    pub estimated_latency_ms: f64,
    pub estimated_energy_mj: f64,
    /// Priority and freshness adjusted application utility per transmitted byte.
    pub utility_per_byte: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PolicyDropReason {
    InvalidMetric,
    Unauthenticated,
    UntrustedSource,
    StaleInput,
    DeadlineExpired,
    DeadlineUnreachable,
    UnknownAction,
    ForbiddenAction,
    ExcessiveAuthority,
    ProposalOutOfBounds,
    ManifestNotApproved,
    TrafficClassMismatch,
    LinkTooDegraded,
    EnergyBudgetExceeded,
    ResidualNotValidated,
    InsufficientUtility,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DeliveryDecision {
    Send(DeliveryPlan),
    Drop(PolicyDropReason),
}

impl DeliveryDecision {
    pub fn plan(self) -> Option<DeliveryPlan> {
        match self {
            Self::Send(plan) => Some(plan),
            Self::Drop(_) => None,
        }
    }
}

/// Indexed result from [`AdaptivePolicy::schedule`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScheduledDecision {
    pub original_index: usize,
    pub decision: DeliveryDecision,
}

/// Tunable transport policy.  Defaults are conservative starting values and
/// should be validated against hardware energy and link traces before flight.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptivePolicyConfig {
    pub capability: CapabilityGateConfig,
    /// A local link sample older than 2 s is not used.
    pub max_link_snapshot_age_ms: u64,
    /// Keep at least 5,000 mJ unavailable to mesh traffic.
    pub minimum_energy_reserve_mj: f64,
    /// Any one transmission may consume at most 1% of remaining energy.
    pub max_message_energy_fraction: f64,
    /// Smallest priority/freshness-adjusted utility per transmitted byte.
    pub minimum_utility_per_byte: f64,
    /// Learned evidence expires after 24 h by default.
    pub max_residual_evidence_age_ms: u64,
    pub residual_min_samples: u32,
    pub residual_min_delta_utility: f64,
    pub residual_max_p_value: f64,
    pub residual_max_relative_error: f64,
    pub residual_min_task_success_delta: f64,
}

impl Default for AdaptivePolicyConfig {
    fn default() -> Self {
        Self {
            capability: CapabilityGateConfig::default(),
            max_link_snapshot_age_ms: 2_000,
            minimum_energy_reserve_mj: 5_000.0,
            max_message_energy_fraction: 0.01,
            minimum_utility_per_byte: 0.000_001,
            max_residual_evidence_age_ms: 86_400_000,
            residual_min_samples: 30,
            residual_min_delta_utility: 0.0,
            residual_max_p_value: 0.05,
            residual_max_relative_error: 0.35,
            residual_min_task_success_delta: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AdaptivePolicy {
    config: AdaptivePolicyConfig,
    gate: CapabilityGate,
}

impl AdaptivePolicy {
    pub fn new(config: AdaptivePolicyConfig) -> Self {
        Self {
            gate: CapabilityGate::new(config.capability),
            config,
        }
    }

    pub fn config(&self) -> &AdaptivePolicyConfig {
        &self.config
    }

    /// Local control-plane seam.  Calling this is an operator/mission-loader
    /// action, never a consequence of receiving a LatentMesh message.
    pub fn approve_manifest(&mut self, manifest_id: u64) -> bool {
        self.gate.approve_manifest(manifest_id)
    }

    pub fn revoke_manifest(&mut self, manifest_id: u64) -> bool {
        self.gate.revoke_manifest(manifest_id)
    }

    /// Evaluate one message without reading a clock or mutable global state.
    pub fn evaluate(
        &self,
        now_ms: u64,
        link: &LinkSnapshot,
        intent: &MessageIntent,
    ) -> DeliveryDecision {
        if !link.valid()
            || !intent.utility.is_finite()
            || !(0.0..=1.0).contains(&intent.utility)
            || !intent.encode_energy_mj.is_finite()
            || intent.encode_energy_mj < 0.0
            || !self.config_values_valid()
            || intent.logical_bytes == 0
            || intent.wire_bytes == 0
        {
            return DeliveryDecision::Drop(PolicyDropReason::InvalidMetric);
        }
        if link.observed_at_ms > now_ms.saturating_add(self.config.capability.max_clock_skew_ms)
            || now_ms.saturating_sub(link.observed_at_ms) > self.config.max_link_snapshot_age_ms
        {
            return DeliveryDecision::Drop(PolicyDropReason::StaleInput);
        }
        if intent.deadline_at_ms < intent.created_at_ms
            || intent.created_at_ms
                > now_ms.saturating_add(self.config.capability.max_clock_skew_ms)
            || now_ms.saturating_sub(intent.created_at_ms) > intent.traffic_class.max_age_ms()
        {
            return DeliveryDecision::Drop(PolicyDropReason::StaleInput);
        }
        if now_ms > intent.deadline_at_ms {
            return DeliveryDecision::Drop(PolicyDropReason::DeadlineExpired);
        }

        let capability = match self.gate.authorize(now_ms, &intent.security, intent.action) {
            Ok(capability) => capability,
            Err(reason) => return DeliveryDecision::Drop(reason),
        };
        if !intent.traffic_class.permits(intent.action) {
            return DeliveryDecision::Drop(PolicyDropReason::TrafficClassMismatch);
        }
        if let Err(reason) = self.validate_residual(now_ms, intent) {
            return DeliveryDecision::Drop(reason);
        }

        let (ack, redundancy) =
            match self.delivery_treatment(link.packet_loss, intent.traffic_class) {
                Some(treatment) => treatment,
                None => return DeliveryDecision::Drop(PolicyDropReason::LinkTooDegraded),
            };
        let estimated_wire_bytes = expanded_wire_bytes(intent.wire_bytes, redundancy);
        let transmit_ms = estimated_wire_bytes as f64 * 8_000.0 / link.throughput_bps;
        let ack_ms = if ack == AckMode::Required {
            link.rtt_ms
        } else {
            0.0
        };
        let estimated_latency_ms = link.queue_delay_ms + link.rtt_ms / 2.0 + transmit_ms + ack_ms;
        if !estimated_latency_ms.is_finite()
            || now_ms.saturating_add(estimated_latency_ms.ceil() as u64) > intent.deadline_at_ms
        {
            return DeliveryDecision::Drop(PolicyDropReason::DeadlineUnreachable);
        }

        let estimated_energy_mj =
            intent.encode_energy_mj + estimated_wire_bytes as f64 * link.energy_per_byte_mj;
        let energy_available =
            (link.energy_remaining_mj - self.config.minimum_energy_reserve_mj).max(0.0);
        let per_message_limit = link.energy_remaining_mj * self.config.max_message_energy_fraction;
        if !estimated_energy_mj.is_finite()
            || estimated_energy_mj > energy_available
            || estimated_energy_mj > per_message_limit
        {
            return DeliveryDecision::Drop(PolicyDropReason::EnergyBudgetExceeded);
        }

        let age_ms = now_ms.saturating_sub(intent.created_at_ms);
        let density = match utility_per_byte(
            intent.utility,
            intent.traffic_class,
            age_ms,
            estimated_wire_bytes,
        ) {
            Ok(density) => density,
            Err(reason) => return DeliveryDecision::Drop(reason),
        };
        if density < self.config.minimum_utility_per_byte {
            return DeliveryDecision::Drop(PolicyDropReason::InsufficientUtility);
        }

        DeliveryDecision::Send(DeliveryPlan {
            capability,
            ack,
            redundancy,
            estimated_wire_bytes,
            estimated_latency_ms,
            estimated_energy_mj,
            utility_per_byte: density,
        })
    }

    /// Evaluate and rank candidates by utility per actual transmitted byte.
    /// Send decisions precede drops; ties preserve caller order.
    pub fn schedule(
        &self,
        now_ms: u64,
        link: &LinkSnapshot,
        intents: &[MessageIntent],
    ) -> Vec<ScheduledDecision> {
        let mut decisions: Vec<_> = intents
            .iter()
            .enumerate()
            .map(|(original_index, intent)| ScheduledDecision {
                original_index,
                decision: self.evaluate(now_ms, link, intent),
            })
            .collect();
        decisions.sort_by(|a, b| match (a.decision, b.decision) {
            (DeliveryDecision::Send(a_plan), DeliveryDecision::Send(b_plan)) => b_plan
                .utility_per_byte
                .partial_cmp(&a_plan.utility_per_byte)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.original_index.cmp(&b.original_index)),
            (DeliveryDecision::Send(_), DeliveryDecision::Drop(_)) => Ordering::Less,
            (DeliveryDecision::Drop(_), DeliveryDecision::Send(_)) => Ordering::Greater,
            (DeliveryDecision::Drop(_), DeliveryDecision::Drop(_)) => {
                a.original_index.cmp(&b.original_index)
            }
        });
        decisions
    }

    fn config_values_valid(&self) -> bool {
        self.config.minimum_energy_reserve_mj.is_finite()
            && self.config.minimum_energy_reserve_mj >= 0.0
            && self.config.max_message_energy_fraction.is_finite()
            && self.config.max_message_energy_fraction > 0.0
            && self.config.max_message_energy_fraction <= 1.0
            && self.config.minimum_utility_per_byte.is_finite()
            && self.config.minimum_utility_per_byte >= 0.0
            && self.config.residual_min_delta_utility.is_finite()
            && self.config.residual_max_p_value.is_finite()
            && (0.0..=1.0).contains(&self.config.residual_max_p_value)
            && self.config.residual_max_relative_error.is_finite()
            && self.config.residual_max_relative_error >= 0.0
            && self.config.residual_min_task_success_delta.is_finite()
    }

    fn validate_residual(
        &self,
        now_ms: u64,
        intent: &MessageIntent,
    ) -> Result<(), PolicyDropReason> {
        match (intent.traffic_class, intent.residual) {
            (TrafficClass::LearnedResidual, Some(evidence)) => {
                if !evidence.valid_metrics() {
                    return Err(PolicyDropReason::InvalidMetric);
                }
                if evidence.evaluated_at_ms
                    > now_ms.saturating_add(self.config.capability.max_clock_skew_ms)
                    || now_ms.saturating_sub(evidence.evaluated_at_ms)
                        > self.config.max_residual_evidence_age_ms
                    || !evidence.causal_controls_passed
                    || evidence.sample_count < self.config.residual_min_samples
                    || evidence.measured_delta_utility <= self.config.residual_min_delta_utility
                    || !(0.0..=self.config.residual_max_p_value).contains(&evidence.p_value)
                    || evidence.heldout_relative_residual > self.config.residual_max_relative_error
                    || evidence.task_success_delta < self.config.residual_min_task_success_delta
                    || !evidence.model_compatible
                    || !evidence.decoder_trusted
                    || !evidence.semantic_fallback_available
                {
                    return Err(PolicyDropReason::ResidualNotValidated);
                }
                Ok(())
            }
            (TrafficClass::LearnedResidual, None) => Err(PolicyDropReason::ResidualNotValidated),
            (_, Some(_)) => Err(PolicyDropReason::TrafficClassMismatch),
            (_, None) => Ok(()),
        }
    }

    fn delivery_treatment(&self, loss: f64, class: TrafficClass) -> Option<(AckMode, Redundancy)> {
        let consequential = matches!(
            class,
            TrafficClass::CriticalCoordination | TrafficClass::CooperativeTask
        );
        if loss >= 0.80 || (loss >= 0.65 && !consequential) {
            return None;
        }
        let redundancy = if loss >= 0.50 {
            Redundancy {
                copies: 3,
                parity_percent: 50,
            }
        } else if loss >= 0.25 {
            Redundancy {
                copies: 2,
                parity_percent: 35,
            }
        } else if loss >= 0.10 {
            Redundancy {
                copies: 1,
                parity_percent: 20,
            }
        } else {
            Redundancy {
                copies: 1,
                parity_percent: 0,
            }
        };
        let ack = if consequential || loss >= 0.25 {
            AckMode::Required
        } else if matches!(
            class,
            TrafficClass::StateSync | TrafficClass::SnapshotRequest
        ) || loss >= 0.10
        {
            AckMode::Opportunistic
        } else {
            AckMode::None
        };
        Some((ack, redundancy))
    }
}

impl Default for AdaptivePolicy {
    fn default() -> Self {
        Self::new(AdaptivePolicyConfig::default())
    }
}

/// Priority/freshness-adjusted utility divided by actual transmitted bytes.
pub fn utility_per_byte(
    utility: f64,
    class: TrafficClass,
    age_ms: u64,
    transmitted_bytes: u64,
) -> Result<f64, PolicyDropReason> {
    if !utility.is_finite() || !(0.0..=1.0).contains(&utility) || transmitted_bytes == 0 {
        return Err(PolicyDropReason::InvalidMetric);
    }
    if age_ms > class.max_age_ms() {
        return Err(PolicyDropReason::StaleInput);
    }
    let freshness = 1.0 - age_ms as f64 / class.max_age_ms() as f64;
    Ok(utility * class.priority_weight() * freshness / transmitted_bytes as f64)
}

fn expanded_wire_bytes(base: u32, redundancy: Redundancy) -> u64 {
    let numerator = u64::from(base)
        .saturating_mul(u64::from(redundancy.copies))
        .saturating_mul(100 + u64::from(redundancy.parity_percent));
    numerator.saturating_add(99) / 100
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 1_000_000;

    fn context(authority: AuthorityLevel) -> SecurityContext {
        SecurityContext {
            authenticated: true,
            trusted_source: true,
            issued_at_ms: NOW - 100,
            expires_at_ms: NOW + 10_000,
            requested_authority: authority,
        }
    }

    fn link(loss: f64) -> LinkSnapshot {
        LinkSnapshot {
            observed_at_ms: NOW - 10,
            rtt_ms: 40.0,
            packet_loss: loss,
            throughput_bps: 1_000_000.0,
            queue_delay_ms: 5.0,
            energy_per_byte_mj: 0.001,
            energy_remaining_mj: 100_000.0,
        }
    }

    fn intent(class: TrafficClass, action: RequestedAction) -> MessageIntent {
        MessageIntent {
            traffic_class: class,
            logical_bytes: 256,
            wire_bytes: 128,
            utility: 0.8,
            created_at_ms: NOW - 100,
            deadline_at_ms: NOW + 2_000,
            encode_energy_mj: 0.2,
            security: context(AuthorityLevel::ObserveOnly),
            action,
            residual: None,
        }
    }

    #[test]
    fn capability_gate_allows_only_the_five_coordination_capabilities() {
        let mut gate = CapabilityGate::default();
        assert!(gate.approve_manifest(7));
        let allowed = [
            RequestedAction::Observe,
            RequestedAction::StateSync,
            RequestedAction::SnapshotRequest,
            RequestedAction::NonbindingAdvisory,
            RequestedAction::CooperativeTaskProposal {
                manifest_id: 7,
                participant_count: 4,
                ttl_ms: 10_000,
                effort_units: 100,
            },
        ];
        for action in allowed {
            let authority = if matches!(
                action,
                RequestedAction::NonbindingAdvisory
                    | RequestedAction::CooperativeTaskProposal { .. }
            ) {
                AuthorityLevel::MissionCoordination
            } else {
                AuthorityLevel::ObserveOnly
            };
            assert!(gate.authorize(NOW, &context(authority), action).is_ok());
        }
    }

    #[test]
    fn every_flight_or_actuator_action_is_categorically_rejected() {
        let gate = CapabilityGate::default();
        let forbidden = [
            RequestedAction::ActuatorCommand,
            RequestedAction::Arm,
            RequestedAction::Disarm,
            RequestedAction::FlightModeChange,
            RequestedAction::AttitudeSetpoint,
            RequestedAction::RateSetpoint,
            RequestedAction::VelocitySetpoint,
            RequestedAction::PositionSetpoint,
            RequestedAction::GeofenceOverride,
            RequestedAction::FailsafeOverride,
            RequestedAction::PayloadRelease,
        ];
        for action in forbidden {
            assert_eq!(
                gate.authorize(NOW, &context(AuthorityLevel::ObserveOnly), action),
                Err(PolicyDropReason::ForbiddenAction)
            );
        }
    }

    #[test]
    fn unknown_excessive_untrusted_and_stale_inputs_fail_closed() {
        let gate = CapabilityGate::default();
        assert_eq!(
            gate.authorize(
                NOW,
                &context(AuthorityLevel::ObserveOnly),
                RequestedAction::Unknown(77)
            ),
            Err(PolicyDropReason::UnknownAction)
        );
        assert_eq!(
            gate.authorize(
                NOW,
                &context(AuthorityLevel::FlightAuthority),
                RequestedAction::NonbindingAdvisory
            ),
            Err(PolicyDropReason::ExcessiveAuthority)
        );
        let mut bad = context(AuthorityLevel::ObserveOnly);
        bad.authenticated = false;
        assert_eq!(
            gate.authorize(NOW, &bad, RequestedAction::Observe),
            Err(PolicyDropReason::Unauthenticated)
        );
        bad = context(AuthorityLevel::ObserveOnly);
        bad.trusted_source = false;
        assert_eq!(
            gate.authorize(NOW, &bad, RequestedAction::Observe),
            Err(PolicyDropReason::UntrustedSource)
        );
        bad = context(AuthorityLevel::ObserveOnly);
        bad.issued_at_ms = NOW - 31_000;
        assert_eq!(
            gate.authorize(NOW, &bad, RequestedAction::Observe),
            Err(PolicyDropReason::StaleInput)
        );
    }

    #[test]
    fn serialized_input_cannot_self_assert_authentication_or_trust() {
        let wire = format!(
            "authenticated = true\ntrusted_source = true\nissued_at_ms = {}\nexpires_at_ms = {}\nrequested_authority = 'ObserveOnly'\n",
            NOW - 1,
            NOW + 1_000
        );
        let decoded: SecurityContext = toml::from_str(&wire).unwrap();
        assert!(!decoded.authenticated);
        assert!(!decoded.trusted_source);
        assert_eq!(
            CapabilityGate::default().authorize(NOW, &decoded, RequestedAction::Observe),
            Err(PolicyDropReason::Unauthenticated)
        );
    }

    #[test]
    fn bounded_proposals_are_enforced() {
        let mut gate = CapabilityGate::default();
        assert!(gate.approve_manifest(7));
        let action = RequestedAction::CooperativeTaskProposal {
            manifest_id: 7,
            participant_count: 65,
            ttl_ms: 1,
            effort_units: 1,
        };
        assert_eq!(
            gate.authorize(NOW, &context(AuthorityLevel::MissionCoordination), action),
            Err(PolicyDropReason::ProposalOutOfBounds)
        );
    }

    #[test]
    fn cooperative_proposals_require_a_locally_approved_manifest() {
        let mut gate = CapabilityGate::default();
        let action = RequestedAction::CooperativeTaskProposal {
            manifest_id: 41,
            participant_count: 4,
            ttl_ms: 1_000,
            effort_units: 10,
        };
        assert_eq!(
            gate.authorize(NOW, &context(AuthorityLevel::MissionCoordination), action),
            Err(PolicyDropReason::ManifestNotApproved)
        );
        assert!(gate.approve_manifest(41));
        assert_eq!(
            gate.authorize(NOW, &context(AuthorityLevel::MissionCoordination), action),
            Ok(AuthorizedCapability::CooperativeTaskProposal)
        );
        assert!(gate.revoke_manifest(41));
        assert_eq!(
            gate.authorize(NOW, &context(AuthorityLevel::MissionCoordination), action),
            Err(PolicyDropReason::ManifestNotApproved)
        );
        assert!(!gate.approve_manifest(0));
    }

    #[test]
    fn valid_state_sync_is_sent() {
        let decision = AdaptivePolicy::default().evaluate(
            NOW,
            &link(0.0),
            &intent(TrafficClass::StateSync, RequestedAction::StateSync),
        );
        let plan = decision.plan().expect("valid state sync should send");
        assert_eq!(plan.ack, AckMode::Opportunistic);
        assert_eq!(plan.redundancy.copies, 1);
        assert_eq!(plan.estimated_wire_bytes, 128);
    }

    #[test]
    fn degraded_link_adds_ack_and_redundancy() {
        let mut message = intent(
            TrafficClass::CriticalCoordination,
            RequestedAction::NonbindingAdvisory,
        );
        message.security.requested_authority = AuthorityLevel::MissionCoordination;
        let plan = AdaptivePolicy::default()
            .evaluate(NOW, &link(0.55), &message)
            .plan()
            .expect("critical coordination remains eligible on degraded link");
        assert_eq!(plan.ack, AckMode::Required);
        assert_eq!(plan.redundancy.copies, 3);
        assert_eq!(plan.redundancy.parity_percent, 50);
        assert_eq!(plan.estimated_wire_bytes, 576);
    }

    #[test]
    fn degraded_link_suppresses_low_priority_traffic() {
        let decision = AdaptivePolicy::default().evaluate(
            NOW,
            &link(0.70),
            &intent(TrafficClass::Observation, RequestedAction::Observe),
        );
        assert_eq!(
            decision,
            DeliveryDecision::Drop(PolicyDropReason::LinkTooDegraded)
        );
    }

    #[test]
    fn stale_and_unreachable_deadlines_are_dropped() {
        let policy = AdaptivePolicy::default();
        let mut stale = intent(TrafficClass::Observation, RequestedAction::Observe);
        stale.created_at_ms = NOW - 6_000;
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &stale),
            DeliveryDecision::Drop(PolicyDropReason::StaleInput)
        );
        let mut impossible = intent(TrafficClass::Observation, RequestedAction::Observe);
        impossible.deadline_at_ms = NOW + 1;
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &impossible),
            DeliveryDecision::Drop(PolicyDropReason::DeadlineUnreachable)
        );
    }

    #[test]
    fn non_finite_link_or_utility_fails_closed() {
        let policy = AdaptivePolicy::default();
        let mut bad_link = link(0.0);
        bad_link.packet_loss = f64::NAN;
        assert_eq!(
            policy.evaluate(
                NOW,
                &bad_link,
                &intent(TrafficClass::Observation, RequestedAction::Observe)
            ),
            DeliveryDecision::Drop(PolicyDropReason::InvalidMetric)
        );
        let mut bad_message = intent(TrafficClass::Observation, RequestedAction::Observe);
        bad_message.utility = f64::INFINITY;
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &bad_message),
            DeliveryDecision::Drop(PolicyDropReason::InvalidMetric)
        );
    }

    #[test]
    fn learned_residual_requires_fresh_causal_evidence_and_fallback() {
        let policy = AdaptivePolicy::default();
        let mut residual = intent(TrafficClass::LearnedResidual, RequestedAction::Observe);
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &residual),
            DeliveryDecision::Drop(PolicyDropReason::ResidualNotValidated)
        );
        residual.residual = Some(ResidualEvidence {
            evaluated_at_ms: NOW - 1_000,
            causal_controls_passed: true,
            sample_count: 100,
            measured_delta_utility: 0.05,
            p_value: 0.01,
            heldout_relative_residual: 0.20,
            task_success_delta: 0.01,
            model_compatible: true,
            decoder_trusted: true,
            semantic_fallback_available: true,
        });
        assert!(policy.evaluate(NOW, &link(0.0), &residual).plan().is_some());
        residual.residual.as_mut().unwrap().task_success_delta = -0.01;
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &residual),
            DeliveryDecision::Drop(PolicyDropReason::ResidualNotValidated)
        );
        residual.residual.as_mut().unwrap().task_success_delta = 0.01;
        residual.residual.as_mut().unwrap().p_value = f64::NAN;
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &residual),
            DeliveryDecision::Drop(PolicyDropReason::InvalidMetric)
        );
    }

    #[test]
    fn an_expired_message_uses_the_explicit_deadline_drop() {
        let mut expired = intent(TrafficClass::Observation, RequestedAction::Observe);
        expired.deadline_at_ms = NOW - 1;
        assert_eq!(
            AdaptivePolicy::default().evaluate(NOW, &link(0.0), &expired),
            DeliveryDecision::Drop(PolicyDropReason::DeadlineExpired)
        );
    }

    #[test]
    fn energy_reserve_suppresses_transmission() {
        let mut exhausted = link(0.0);
        exhausted.energy_remaining_mj = 5_000.0;
        assert_eq!(
            AdaptivePolicy::default().evaluate(
                NOW,
                &exhausted,
                &intent(TrafficClass::Observation, RequestedAction::Observe)
            ),
            DeliveryDecision::Drop(PolicyDropReason::EnergyBudgetExceeded)
        );
    }

    #[test]
    fn scheduler_orders_sendable_messages_by_utility_density() {
        let policy = AdaptivePolicy::default();
        let mut large = intent(TrafficClass::Observation, RequestedAction::Observe);
        large.wire_bytes = 1_024;
        let small = intent(TrafficClass::Observation, RequestedAction::Observe);
        let scheduled = policy.schedule(NOW, &link(0.0), &[large, small]);
        assert_eq!(scheduled[0].original_index, 1);
        assert_eq!(scheduled[1].original_index, 0);
    }

    #[test]
    fn priority_label_cannot_bypass_action_class_or_authority() {
        let policy = AdaptivePolicy::default();
        let mislabeled = intent(TrafficClass::CriticalCoordination, RequestedAction::Observe);
        assert_eq!(
            policy.evaluate(NOW, &link(0.0), &mislabeled),
            DeliveryDecision::Drop(PolicyDropReason::TrafficClassMismatch)
        );
    }
}
