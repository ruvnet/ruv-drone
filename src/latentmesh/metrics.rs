//! Lock-free, payload-agnostic counters for a LatentMesh node.
//!
//! Counters intentionally accept only sizes and event categories.  They do not
//! retain peer identifiers, payloads, signatures, tokens, or key material.

use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};

use super::policy::PolicyDropReason;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DropCounter {
    Authentication,
    Replay,
    Stale,
    Policy,
}

/// Copyable point-in-time view suitable for telemetry or tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct MetricsSnapshot {
    pub sent_messages: u64,
    pub received_messages: u64,
    pub sent_logical_bytes: u64,
    pub sent_wire_bytes: u64,
    pub received_logical_bytes: u64,
    pub received_wire_bytes: u64,
    pub authentication_drops: u64,
    pub replay_drops: u64,
    pub stale_drops: u64,
    pub policy_drops: u64,
    pub reassemblies_completed: u64,
    pub reassembly_failures: u64,
    pub reassembled_fragments: u64,
    pub residual_suppressions: u64,
}

/// Relaxed atomics are sufficient: each value is a monotonic statistic, not a
/// synchronization primitive.  Updates saturate at `u64::MAX` instead of
/// wrapping and producing misleadingly small security counters.
#[derive(Debug, Default)]
pub struct LatentMeshMetrics {
    sent_messages: AtomicU64,
    received_messages: AtomicU64,
    sent_logical_bytes: AtomicU64,
    sent_wire_bytes: AtomicU64,
    received_logical_bytes: AtomicU64,
    received_wire_bytes: AtomicU64,
    authentication_drops: AtomicU64,
    replay_drops: AtomicU64,
    stale_drops: AtomicU64,
    policy_drops: AtomicU64,
    reassemblies_completed: AtomicU64,
    reassembly_failures: AtomicU64,
    reassembled_fragments: AtomicU64,
    residual_suppressions: AtomicU64,
}

impl LatentMeshMetrics {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_send(&self, logical_bytes: u64, wire_bytes: u64) {
        saturating_add(&self.sent_messages, 1);
        saturating_add(&self.sent_logical_bytes, logical_bytes);
        saturating_add(&self.sent_wire_bytes, wire_bytes);
    }

    pub fn record_receive(&self, logical_bytes: u64, wire_bytes: u64) {
        saturating_add(&self.received_messages, 1);
        saturating_add(&self.received_logical_bytes, logical_bytes);
        saturating_add(&self.received_wire_bytes, wire_bytes);
    }

    pub fn record_drop(&self, kind: DropCounter) {
        let counter = match kind {
            DropCounter::Authentication => &self.authentication_drops,
            DropCounter::Replay => &self.replay_drops,
            DropCounter::Stale => &self.stale_drops,
            DropCounter::Policy => &self.policy_drops,
        };
        saturating_add(counter, 1);
    }

    /// Classify a policy rejection into the stable operational counters.  A
    /// residual rejection increments both the policy and residual counters so
    /// suppression remains visible without recording residual contents.
    pub fn record_policy_drop(&self, reason: PolicyDropReason) {
        let kind = match reason {
            PolicyDropReason::Unauthenticated | PolicyDropReason::UntrustedSource => {
                DropCounter::Authentication
            }
            PolicyDropReason::StaleInput | PolicyDropReason::DeadlineExpired => DropCounter::Stale,
            _ => DropCounter::Policy,
        };
        self.record_drop(kind);
        if reason == PolicyDropReason::ResidualNotValidated {
            self.record_residual_suppressed();
        }
    }

    /// Record successful reconstruction of one logical message.
    pub fn record_reassembly_completed(&self, fragment_count: u64) {
        saturating_add(&self.reassemblies_completed, 1);
        saturating_add(&self.reassembled_fragments, fragment_count);
    }

    pub fn record_reassembly_failure(&self) {
        saturating_add(&self.reassembly_failures, 1);
    }

    pub fn record_residual_suppressed(&self) {
        saturating_add(&self.residual_suppressions, 1);
    }

    pub fn snapshot(&self) -> MetricsSnapshot {
        MetricsSnapshot {
            sent_messages: load(&self.sent_messages),
            received_messages: load(&self.received_messages),
            sent_logical_bytes: load(&self.sent_logical_bytes),
            sent_wire_bytes: load(&self.sent_wire_bytes),
            received_logical_bytes: load(&self.received_logical_bytes),
            received_wire_bytes: load(&self.received_wire_bytes),
            authentication_drops: load(&self.authentication_drops),
            replay_drops: load(&self.replay_drops),
            stale_drops: load(&self.stale_drops),
            policy_drops: load(&self.policy_drops),
            reassemblies_completed: load(&self.reassemblies_completed),
            reassembly_failures: load(&self.reassembly_failures),
            reassembled_fragments: load(&self.reassembled_fragments),
            residual_suppressions: load(&self.residual_suppressions),
        }
    }
}

fn load(counter: &AtomicU64) -> u64 {
    counter.load(Ordering::Relaxed)
}

fn saturating_add(counter: &AtomicU64, value: u64) {
    let _ = counter.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |current| {
        Some(current.saturating_add(value))
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    #[test]
    fn snapshot_starts_at_zero_and_tracks_wire_vs_logical_bytes() {
        let metrics = LatentMeshMetrics::new();
        assert_eq!(metrics.snapshot(), MetricsSnapshot::default());
        metrics.record_send(100, 70);
        metrics.record_receive(200, 140);
        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.sent_messages, 1);
        assert_eq!(snapshot.received_messages, 1);
        assert_eq!(snapshot.sent_logical_bytes, 100);
        assert_eq!(snapshot.sent_wire_bytes, 70);
        assert_eq!(snapshot.received_logical_bytes, 200);
        assert_eq!(snapshot.received_wire_bytes, 140);
    }

    #[test]
    fn security_policy_and_reassembly_events_are_distinct() {
        let metrics = LatentMeshMetrics::new();
        metrics.record_drop(DropCounter::Authentication);
        metrics.record_drop(DropCounter::Replay);
        metrics.record_drop(DropCounter::Stale);
        metrics.record_drop(DropCounter::Policy);
        metrics.record_reassembly_completed(4);
        metrics.record_reassembly_failure();
        metrics.record_residual_suppressed();
        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.authentication_drops, 1);
        assert_eq!(snapshot.replay_drops, 1);
        assert_eq!(snapshot.stale_drops, 1);
        assert_eq!(snapshot.policy_drops, 1);
        assert_eq!(snapshot.reassemblies_completed, 1);
        assert_eq!(snapshot.reassembly_failures, 1);
        assert_eq!(snapshot.reassembled_fragments, 4);
        assert_eq!(snapshot.residual_suppressions, 1);
    }

    #[test]
    fn policy_reasons_are_classified_without_payload_context() {
        let metrics = LatentMeshMetrics::new();
        metrics.record_policy_drop(PolicyDropReason::Unauthenticated);
        metrics.record_policy_drop(PolicyDropReason::StaleInput);
        metrics.record_policy_drop(PolicyDropReason::ResidualNotValidated);
        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.authentication_drops, 1);
        assert_eq!(snapshot.stale_drops, 1);
        assert_eq!(snapshot.policy_drops, 1);
        assert_eq!(snapshot.residual_suppressions, 1);
    }

    #[test]
    fn counters_are_safe_under_concurrent_recording() {
        let metrics = Arc::new(LatentMeshMetrics::new());
        let workers: Vec<_> = (0..4)
            .map(|_| {
                let metrics = Arc::clone(&metrics);
                thread::spawn(move || {
                    for _ in 0..1_000 {
                        metrics.record_send(10, 12);
                        metrics.record_drop(DropCounter::Policy);
                    }
                })
            })
            .collect();
        for worker in workers {
            worker.join().unwrap();
        }
        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.sent_messages, 4_000);
        assert_eq!(snapshot.sent_logical_bytes, 40_000);
        assert_eq!(snapshot.sent_wire_bytes, 48_000);
        assert_eq!(snapshot.policy_drops, 4_000);
    }

    #[test]
    fn counters_saturate_instead_of_wrapping() {
        let counter = AtomicU64::new(u64::MAX - 1);
        saturating_add(&counter, 10);
        assert_eq!(counter.load(Ordering::Relaxed), u64::MAX);
    }

    #[test]
    fn snapshot_debug_surface_contains_counts_not_sensitive_context() {
        let metrics = LatentMeshMetrics::new();
        metrics.record_send(7, 9);
        let debug = format!("{:?}", metrics.snapshot());
        assert!(debug.contains("sent_wire_bytes"));
        assert!(!debug.contains("payload"));
        assert!(!debug.contains("peer"));
        assert!(!debug.contains("token"));
        assert!(!debug.contains("key"));
    }
}
