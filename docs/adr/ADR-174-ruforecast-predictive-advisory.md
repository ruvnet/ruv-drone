# ADR-174: RuForecast predictive advisory

| | |
|---|---|
| **Status** | Accepted; baseline shadow integration implemented behind an opt-in feature |
| **Date** | 2026-09-02 |
| **Deciders** | ruv-drone maintainers |
| **Security review** | [RuForecast integration threat model](../security/ruforecast-threat-model.md) |
| **Upstream pin** | `ruvnet/RuForecast@59173b55ef51a122db222c9902aeae521713d0a5` |

## Decision

Integrate only `ruforecast-core` as an optional exact-revision dependency. The
first release uses its deterministic last-value baseline, validated time-series
contract, quantiles, provenance, privacy policy, abstention, and content
receipts. It does not link the Burn model or a hosted training runtime.

The forecast module observes only local battery percentage, normalized link
quality, and aggregate mission progress. History is a fixed-capacity queue,
128 rows by default. Forecast refresh is cadence-limited. Output is an expiring
summary bound to the request and output digests.

The authority invariant is:

> A forecast may reduce eligibility for new cooperative work only. It cannot
> create work, control flight, change topology, override a geofence, or enter,
> leave, or clear a fail-safe.

`Shadow` is the default and never reduces eligibility. `CanaryReduceOnly` is a
local opt-in. Missing, invalid, stale, abstaining, or disabled output preserves
existing behavior. This availability choice is safe because existing onboard
fail-safes remain authoritative and independent.

## Context and evidence

The practical use cases are predicting battery margin before another task,
detecting a weakening link before assigning relay work, and estimating mission
progress. Forecasting position or velocity is excluded because it could be
mistaken for collision-avoidance or control evidence.

RuForecast's own evidence ledger states that no learned configuration has yet
reliably beaten last-value and seasonal-naive baselines out of sample. Two
search winners failed independent verification. Treating an unproven network
as an operational improvement would be less capable than an honest baseline.
This ADR therefore makes baseline comparison and abstention prerequisites for
future model activation.

## Component contract

| Component | Input | Output | Authority |
|---|---|---|---|
| Bounded history | Three finite, range-checked local values and monotonic time | Up to 128 observations | None |
| RuForecast core | Typed series, policy, horizon, cadence, quantiles | Forecast or abstention with receipts | None |
| Shadow evaluator | Forecast summary and counters | Evidence only | None |
| Canary gate | Fresh summary and local thresholds | Eligible / ineligible for new work | Reduce only |
| Flight and safety code | Existing authoritative state | Existing control and fail-safe behavior | Unchanged |

The module owns no flight-controller, geofence, task-allocation mutation,
`peer_states`, or fail-safe mutation handle. Its orchestrator call occurs after
normal safety and mission processing. The only consumer method is explicitly
named `forecast_is_eligible_for_new_work`.

## Validation and promotion gates

A learned artifact remains shadow-only until all gates pass on independent,
out-of-search, temporally split data:

1. Beat both last-value and seasonal-naive weighted quantile loss on every
   approved corpus by a predeclared margin; no averaged-away corpus failures.
2. Meet per-horizon loss and interval-coverage calibration bounds, including
   missingness, stale input, and distribution-shift slices.
3. Carry an immutable artifact digest, exact feature schema, training-policy
   receipt, dataset provenance, and independent evaluation receipt.
4. Pass activation allowlisting, resource limits, malformed-output tests,
   restart/rollback tests, security review, and flight-authority regression.
5. Meet a declared p50/p95/p99 latency, memory, and energy budget on target
   companion hardware under concurrent mission load.

Promotion stages are `Disabled` → `Shadow baseline` → `Shadow learned
candidate` → `Canary reduce-only` → `Fleet reduce-only`. Every stage has an
immediate local rollback to `Disabled` or a build without the feature.

## LatentMesh relationship

This release keeps forecasts local. A later ADR may define a signed,
versioned, compact LatentMesh advisory containing only horizon, quantiles,
expiry, source-state digest, artifact digest, request digest, and output digest.
The receiver must treat it as an untrusted peer claim with the same reduce-only
authority. Learned residuals and raw history remain forbidden on the wire.

## Alternatives rejected

- **Activate `ruforecast-model` now:** rejected because current upstream
  evidence does not show reliable baseline lift.
- **Forecast kinematics:** rejected to protect the collision/control boundary.
- **Send raw histories through LatentMesh:** rejected for privacy, bandwidth,
  replay surface, and unclear operational benefit.
- **Fail closed on forecast outage:** rejected because forecast availability
  must not disable an otherwise safe drone; onboard safety already fails closed
  at the appropriate control boundary.

## Consequences

The integration is useful immediately for contract validation, telemetry
collection, rollout plumbing, and operational baselines without overstating AI
capability. It adds bounded allocation and hashing cost only when enabled.
Materializing the typed series remains the primary measured optimization target;
forecast refresh is cadence-limited to avoid doing that work every control tick.
