# ADR-172: Jellyfish-Inspired Swarm Behaviors (`ruv-jellyfish`)

| | |
|---|---|
| **Status** | Accepted |
| **Date** | 2026-07-12 |
| **Deciders** | ruv-drone maintainers |
| **Relates to** | ADR-148 (Drone Swarm Control System), ADR-171 (Stage-1 evaluation), ADR-147 (OccWorld environment prior) |
| **Crate** | [`crates/ruv-jellyfish`](../../crates/ruv-jellyfish) |

## Context

`ruv-drone` (ADR-148) coordinates industrial cooperative-UAV fleets: SAR
coverage, infrastructure inspection, agriculture, and emergency telecom relay.
Several of those missions are **endurance-bound rather than speed-bound**:

- **Relay chains** (`relay` profile, 6–20 drones) must *hold station* for hours;
  the mission ends when the first battery does.
- **SAR re-scan loops** spend most of their flight time loitering over
  high-probability cells waiting for CSI dwell windows, not transiting.
- **Agricultural monitoring** wants persistent low-energy presence over a field,
  with the fleet contracting onto anomalies and relaxing back to broad coverage.

The existing behavior stack is geometry-first and thrust-constant:

- `formation::reynolds` / `virtual_structure` / `leader_follower` hold *shapes*,
  but have no notion of an energy budget or of exploiting the wind field.
- `planning::coverage` optimizes *paths* (boustrophedon → Bayesian → triangulate),
  not *loiter placement* for persistent presence.
- `allocation::auction` assigns discrete tasks; it cannot express "the whole
  fleet should densify here and thin out there" as a continuous field.

**Why jellyfish?** Jellyfish are the most energy-efficient swimmers measured —
their cost of transport is the lowest of any metazoan, achieved by a
*pulse-and-drift* gait: a short powered bell contraction followed by a long
passive coast during which the stopping vortex recaptures energy (Gemmell et
al., PNAS 2013). At population scale, jellyfish *blooms* aggregate passively by
riding ocean currents and thermoclines rather than fighting them, densifying
where the medium concentrates them and dispersing when it doesn't.

Both traits are exactly the ones our endurance-bound missions lack: a gait that
buys airtime, and an aggregation rule that concentrates presence over what
matters while exploiting the wind field instead of fighting it.

## Decision

Add a **standalone workspace crate, `crates/ruv-jellyfish`**, implementing two
jellyfish-derived behavior primitives and a controller that unifies them around
a single per-vehicle energy budget:

1. A **pulse-and-drift gait** — an analytic energy model plus a real-time state
   machine that realizes efficient transit.
2. A **bloom aggregation** rule — a decentralized, local, energy-aware
   station-keeping controller over a scalar *value field* and a vector *flow
   field*.
3. `JellyfishController` — a per-drone façade that debits both cruise and loiter
   from one energy budget, exposing endurance/range estimates.

The crate is **standalone** (no path-dependency back into `ruview-swarm`), uses
its own light `Vec3`, is `std`-only, and builds and tests in the light default
configuration. It sits *beside* the existing behavior stack, not inside it: the
geometry-first modules keep shapes and paths; `ruv-jellyfish` adds an
energy-first layer for the loiter/presence phase of a mission.

### Why a separate crate, not a module

- **Independent testability & build cost.** The behaviors are pure numerics with
  no dependency on the parent's heavy feature surface (mavlink, candle, ort). A
  standalone crate keeps `cargo test -p ruv-jellyfish` fast and hermetic.
- **Clear seam.** The parent depends on it (optionally) through a thin adapter
  that maps `Position3D`/`Velocity3D` ↔ `Vec3`; the boundary is explicit.
- **Reuse.** Endurance-aware loiter is useful to other RuView vehicles, not just
  this coordination layer.

The repository becomes a Cargo workspace: the root package `ruview-swarm` is the
implicit workspace root, with `crates/ruv-jellyfish` as its first member.

## Design

### 1. Pulse-and-drift energetics (`energy.rs`)

We model motion along a heading as a per-unit-mass system with **linear drag**
`a_drag = −k·v` (a free coast decays as `v(t) = v₀·e^{−k t}`). Two regimes spend
actuation energy differently:

| Regime | Momentum supply | Actuation energy per metre |
|--------|-----------------|----------------------------|
| Constant thrust | continuous, countering drag | `e_per_dv · k` |
| Pulse-and-drift | bursts; drift recaptures a fraction `r` | `e_per_dv · k / (1 + r)` |

The pulse-drift result falls out of a steady-state momentum balance: over a
cycle the paid impulse plus the *freely recaptured* impulse must equal the drag
loss, `(1 + r)·Δv_paid = k·v·T_cycle`, so the paid actuation per metre is a
factor `(1 + r)` below constant thrust. Both regimes additionally pay a constant
hotel/avionics load `p_idle`, whose per-metre cost `p_idle / v` amortizes over
ground covered. Loiter is priced separately by

```
station_keeping_power(rel_flow) = e_per_dv · k · rel_flow / (1 + r) + p_idle
```

which is what a drone pays to hold position against a *relative* flow of speed
`rel_flow`. The lever the bloom pulls is `rel_flow`: ride the current, pay less.

`EnergyModel` exposes `actuation_energy_per_metre`, `energy_per_metre(v)`,
`station_keeping_power`, `actuation_saving_fraction = r/(1+r)`,
`range_metres`, and `loiter_endurance_secs`.

### 2. Pulse-and-drift gait state machine (`pulse.rs`)

`PulseDriftGait` turns the model into a control-rate speed profile:

```
Pulse:  accelerate at `pulse_accel` toward `peak_speed`, charging
        actuation energy ∝ Δv delivered
  └─ at peak: add a one-shot recapture bonus  r · (Δv of this pulse)   [free]
Drift:  coast under linear drag (semi-implicit Euler) toward `drift_floor`
  └─ at floor: begin the next Pulse
```

`step(dt)` returns the commanded speed magnitude; `telemetry()` exposes
distance, elapsed time, and the actuation/idle energy split. At steady state the
realized actuation-per-metre lands at or below the closed-form bound regardless
of the peak/floor band chosen — the band trades pulse cadence against
smoothness, not efficiency. `GaitParams::cruise(target_avg)` picks a band around
a desired average speed; the realized average is read back from telemetry.

### 3. Bloom aggregation (`bloom.rs`, `field.rs`)

A **decentralized local rule** each drone runs on its own observed neighbours.
`FlowField` gives the wind/current vector at a point; `ValueField` gives a
scalar of cooperative interest (SAR victim probability, inspection interest,
NDVI anomaly) with a finite-difference or analytic gradient. Per tick the
controller composes:

| Term | Direction | Gated by |
|------|-----------|----------|
| **Gradient climb** | up the value gradient | aggregation weight `a = g/(g+g₀)` |
| **Separation** | away from neighbours inside `min_spacing` | proximity |
| **Cohesion** | toward local centroid | `a` (clump only when there's a peak) |
| **Dispersion** | away from centroid | `1 − a` (spread when field is flat) |

The composed vector is the **desired ground velocity** `v`. It is made
energy-aware by honest wind compensation: to hold `v` over the ground in flow
`f`, the drone must fly airspeed `v − f` (clamped to `max_airspeed`). Aligned
flow shrinks the airspeed — and the energy — while cross/opposing flow grows it.
`relative_flow_speed = |v − f|` feeds `station_keeping_power`. This is where
"riding the current" pays: a bloom aggregating *with* a `ConvergentFlow` keeps
station almost for free, while the same fleet fighting a headwind pays for it —
and the planner sees the difference in the endurance estimate.

The aggregation weight makes the fleet **self-organize**: strong value gradients
pull a tight smack onto the peak (balanced against separation so it never
collapses); a flat field flips the same drones into dispersion for broad
coverage. No mode switch, no central assignment.

### 4. Unified controller (`controller.rs`)

`JellyfishController` holds a gait, a bloom controller, and one energy budget
(J·kg⁻¹). `cruise(dt)` steps the gait and debits the delta; `loiter(...)`
computes the bloom command and debits `station_keeping_power · dt`. A mission
transits efficiently, then blooms; both draw down the same budget, and
`loiter_endurance_secs` / `cruise_range_metres` report what's left. When the
budget depletes, the caller triggers the fleet's existing return-to-home /
failsafe.

## Crate layout

```
crates/ruv-jellyfish/
├── Cargo.toml            — std-only; serde on by default; publish=false
├── README.md
└── src/
    ├── lib.rs            — re-exports + crate docs + doctest
    ├── vec3.rs           — light 3-D vector (own type; ops + fluent methods)
    ├── energy.rs         — EnergyModel, Gait, closed-form energetics
    ├── pulse.rs          — PulseDriftGait state machine + telemetry
    ├── field.rs          — FlowField / ValueField traits + NoFlow, UniformFlow,
    │                        ConvergentFlow, GaussianHotspot, HotspotField
    ├── bloom.rs          — BloomController, BloomParams, BloomCommand, step_fleet
    └── controller.rs     — JellyfishController (unifies gait + bloom + budget)
tests/mission_arc.rs      — end-to-end cruise→bloom within an energy budget
```

## Public API sketch

```rust
use ruv_jellyfish::{JellyfishController, field::*, vec3::Vec3, EnergyModel, Gait};

// Energetics
let m = EnergyModel::default();
let saved = m.actuation_saving_fraction();          // r/(1+r)
let per_m = m.energy_per_metre(Gait::PulseDrift, 6.0);

// Mission arc
let mut d = JellyfishController::with_budget(500_000.0);   // J/kg
let _speed = d.cruise(0.05);                               // transit
let step = d.loiter(pos, &neighbours, &value, &wind, t, 0.1);  // bloom + station-keep
let _left = d.loiter_endurance_secs(step.command.relative_flow_speed);
```

## Integration with `ruv-drone`

The crate is consumed through a thin, explicit adapter at the call site — no
type leakage in either direction:

- `Position3D { x, y, z }` ↔ `Vec3 { x, y, z }` (same NED convention).
- `Velocity3D { vx, vy, vz }` ↔ `Vec3` for commands.
- The `ValueField` is backed by `planning::probability_grid` (the Bayesian
  victim-probability posterior) or by `sensing`/inspection-interest maps; a
  `HotspotField` is a lightweight stand-in for tests and simple missions.
- The `FlowField` is backed by a wind estimate (telemetry / forecast); `NoFlow`
  and `UniformFlow` cover the common cases, `ConvergentFlow` models a
  concentrating thermocline/convergence zone.
- `JellyfishController::depleted()` feeds the existing `failsafe` machine's
  low-energy transitions rather than introducing a new safety path.

Wiring the parent to it (a `jellyfish` feature that pulls the path-dep and adds
the adapter) is a **follow-up**; this ADR lands the crate, its behaviors, and
the workspace. Nothing in the parent build changes except the new `[workspace]`
table.

## Scope & export compliance

This crate stays firmly inside the project's **industrial / civilian
cooperative-UAV** scope (see [`NOTICE`](../../NOTICE)):

- Both behaviors are **cooperative coverage and station-keeping** primitives.
  The bloom aggregates over a *cooperative interest map* — victim probability,
  inspection interest — which is the same class of signal the existing
  `planning::coverage` and `allocation::auction` modules already act on.
- It implements **no** adaptive behavior in response to threats or mission
  objectives in the controlled sense, **no** target acquisition / tracking-to-
  engage / fire control, and **no** weapons or countermeasure integration.
- "Aggregation" here means concentrating *sensing presence* over
  high-probability regions for search and monitoring — not convergence on a
  target for engagement.

Per the U.S. State Department's clarification distinguishing cooperative and
formation operation from military "swarming," the maintainer's assessment is
that these capabilities fall **outside USML Category VIII(h)(12)**. This is not
legal advice or an export determination; final classification remains the
maintainer's / export counsel's responsibility, and any downstream modification
adding adaptive threat/mission-response behavior may change it.

## Alternatives considered

| Alternative | Why not |
|-------------|---------|
| **Extend `formation::reynolds`** with an energy term | Conflates flocking geometry with energetics and the wind field; Reynolds has no notion of a value field or a budget. Keeping them separate keeps both simple. |
| **A new module inside `ruview-swarm`** instead of a crate | Couples fast, hermetic numerics to the parent's heavy optional deps and slows its test loop; loses the clean adapter seam and cross-vehicle reuse. |
| **Metaheuristic "Jellyfish Search" optimizer** (Chou & Truong 2021) | A global black-box optimizer, not a real-time decentralized controller; it doesn't model energy or run per-tick on local neighbour views. The bio-mechanical framing fits this repo's control-loop and endurance goals far better. |
| **Central bloom solver** (fit a density field, assign slots) | Breaks the decentralized consensus/gossip ethos and adds a single point of failure. The local rule self-organizes without one. |
| **Nonlinear (quadratic) drag** in the energy model | More faithful but the closed forms stop being clean, and the `(1+r)` result — the whole point — survives under linear drag. Left as a documented calibration axis. |

## Consequences

**Positive**

- A quantified endurance lever: `(1 + r)` actuation saving on transit, and
  cheaper loiter when the bloom rides the flow — both visible to the planner.
- Decentralized, standalone, fast to test (31 tests: unit + integration +
  doctest), clippy-clean, `std`-only.
- Clean separation from the geometry-first stack; explicit adapter boundary.

**Negative / risks**

- The energy model is analytic and **needs per-airframe calibration**
  (`drag_k`, `e_per_dv`, `recapture`, `idle_power`); defaults are illustrative.
  Pulse-and-drift maps naturally to coast-capable airframes (fixed-wing,
  hybrid); for multirotors "drift" is a reduced-thrust glide/soar and the gait
  is an abstraction over the wind field, not a literal bell pulse.
- Bloom gains (`grad_gain`, spacing, dispersion) are tuned heuristics; field
  quality (probability-grid resolution, wind estimate) bounds real-world
  performance.
- Parent integration (the `jellyfish` feature + adapter) is deferred to a
  follow-up.

## Testing & validation

- **Unit** — vector math edge cases; energy-model relations (pulse-drift ≤
  constant thrust, saving = `r/(1+r)`, station power grows with relative flow,
  riding flow extends endurance); gait phase cycling, bounded speed, steady-
  state cost matching the analytic bound, and robustness to bad params; field
  gradients (analytic vs numeric agreement, superposition); bloom aggregation,
  minimum-spacing preservation, flat-field dispersion, and wind-compensation
  direction.
- **Integration** (`tests/mission_arc.rs`) — a four-drone fleet cruises for 60 s
  (verifying the gait beats the constant-thrust reference), then blooms over a
  hotspot in a steady wind for 90 s, asserting the smack contracts toward the
  hotspot, no drone depletes its budget, and none collapse together.
- **Lint** — `cargo clippy -p ruv-jellyfish --all-targets` is warning-free.

## References

- J. H. Costello, S. P. Colin, et al. — jellyfish locomotion and the low cost of
  transport of the pulse-and-coast gait.
- B. J. Gemmell et al., "Passive energy recapture in jellyfish contributes to
  propulsive advantage over other metazoans," *PNAS* 110(44), 2013.
- C. Reynolds, "Flocks, herds and schools: A distributed behavioral model,"
  *SIGGRAPH* 1987 (the separation/cohesion/dispersion lineage).
- ADR-148 (Drone Swarm Control System), ADR-171 (Stage-1 evaluation),
  ADR-147 (OccWorld environment prior).
- Repository [`NOTICE`](../../NOTICE) — scope & export.