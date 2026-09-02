# RuForecast integration threat model

## Assets and trust boundaries

Protected assets are flight authority, fail-safe state, task admission,
telemetry privacy, forecast integrity, availability, and performance headroom.
Local telemetry is trusted only after finite/range/monotonic validation. A
RuForecast output is structurally validated evidence, not an authority token.
Future remote forecasts remain untrusted even when transport-authenticated.

## STRIDE analysis

| Threat | Example | Implemented control | Residual / deployment control |
|---|---|---|---|
| Spoofing | Remote claim presented as local forecast | No network ingress; engine accepts local scalar values only | Keep future LatentMesh source/key binding local |
| Tampering | Output or request changed after inference | Canonical request and output receipts; payload-integrity verification | Verify artifact allowlist before learned activation |
| Repudiation | Operator cannot reproduce a decision | Model ID, origin, expiry, request/output digests, aggregate counters | Export redacted audit records under retention policy |
| Information disclosure | Raw position/history leaks into metrics | Only battery/link/progress enter history; counters are payload-free | Protect process memory and any optional audit sink |
| Denial of service | Huge history, excessive inference, timestamp churn | Capacity ≤16,384; default 128; range checks; cadence limiting; saturating counters | Benchmark target hardware under mission load |
| Elevation of privilege | Forecast commands motion or clears fail-safe | Module owns no control/safety mutation handle; reduce-only eligibility API | Code-owner review for dependency-direction changes |

## Abuse and failure cases

- Nonfinite and out-of-range observations are rejected without replacing the
  last valid receipt.
- Duplicate or decreasing timestamps are rejected.
- Insufficient history, disabled mode, stale input, model abstention, malformed
  output, or internal error produces no new authority.
- Expired canary advice is ignored. Shadow advice never changes eligibility.
- A low forecast can block only new cooperative work. Existing flight,
  return-to-home, landing, collision avoidance, and local safety continue.
- History is local P1 derived telemetry with explicit purpose and retention.
  No consent, DPA, or export receipt is fabricated.
- The dependency uses an immutable Git revision and `Cargo.lock`; update the
  revision only with a new source/dependency/security review.

## Required security tests

1. Nonfinite/range/monotonic input rejection and bounded history.
2. Receipt payload-integrity verification and deterministic repeated output.
3. Shadow no-op, canary reduce-only, and stale fail-open behavior.
4. Orchestrator equivalence: forecast policy cannot alter motion or fail-safe.
5. Feature-off build and combined LatentMesh/RuForecast build.
6. Dependency audit, secret scan, unsafe-code review, and STRIDE review.

## Out of scope for this release

Learned artifacts, hosted training, remote forecast ingestion, model-weight
distribution, and over-air forecast schemas are not implemented. They require
artifact signature verification, resource isolation, rollback testing, data
governance, and a new ADR before activation.

## Review evidence

The pinned Ruflo 3.25.6 deep, dependency, secrets, and STRIDE scans reported no
findings. `cargo audit` reported no vulnerabilities and one allowed
unmaintained-crate warning, `RUSTSEC-2024-0436` for `paste 1.0.15`. The path is
the existing `nalgebra 0.33.3` → `simba 0.9.1` dependency, not RuForecast.
Track its upstream replacement separately; do not misreport it as a clean
zero-warning audit or as a vulnerability introduced by this integration.
