# ruv-jellyfish

**Jellyfish-inspired, energy-efficient swarm behaviors for cooperative UAV fleets.**

A standalone member of the [`ruv-drone`](../../) workspace (ADR-172). Where the
parent crate's `formation`, `planning`, and `allocation` modules optimize
*shapes* and *paths*, `ruv-jellyfish` optimizes **endurance** and **presence**
for the missions that are battery-bound rather than speed-bound: relay chains
that must hold station for hours, SAR re-scan loops that loiter over
high-probability cells, and persistent agricultural monitoring.

It draws two mechanisms from real jellyfish biomechanics and ecology:

| Behavior | Biology | Module |
|----------|---------|--------|
| **Pulse-and-drift gait** | Lowest cost-of-transport gait measured in any swimmer — a powered bell pulse, a free stopping-vortex recapture, then a passive coast (Gemmell et al., PNAS 2013) | [`pulse`](src/pulse.rs), [`energy`](src/energy.rs) |
| **Bloom aggregation** | A *smack* densifies where the medium concentrates it and disperses where it does not, riding currents rather than fighting them | [`bloom`](src/bloom.rs), [`field`](src/field.rs) |

[`JellyfishController`](src/controller.rs) unifies both around a single
per-kilogram energy budget so a planner can reason about endurance end to end.

## The energy model in one line

Under linear drag, constant thrust pays actuation energy `e_per_dv · k` per
metre; pulse-and-drift pays `e_per_dv · k / (1 + r)`, a factor **(1 + r)**
cheaper, where `r` is the stopping-vortex recapture fraction. Loiter is priced
by `station_keeping_power`, which falls as the drone's airspeed relative to the
flow falls — so a bloom that aggregates *with* a convergent current keeps
station almost for free.

## Example

```rust
use ruv_jellyfish::{
    JellyfishController,
    field::{GaussianHotspot, HotspotField, UniformFlow},
    vec3::Vec3,
};

// A SAR crew loitering over a high-probability cell in a light breeze.
let value = HotspotField::new(vec![GaussianHotspot {
    centre: Vec3::new(120.0, 80.0, 0.0), peak: 1.0, sigma: 35.0,
}]);
let wind = UniformFlow(Vec3::new(1.0, 0.5, 0.0));

let mut drone = JellyfishController::with_budget(500_000.0); // J/kg
let neighbours = [Vec3::new(130.0, 90.0, 0.0)];
let step = drone.loiter(Vec3::new(100.0, 70.0, 0.0), &neighbours, &value, &wind, 0.0, 0.1);

// `step.command.commanded_airspeed` → what to fly; `drone.budget_remaining()` → endurance left.
```

## Design notes

- **Decentralized.** `BloomController` is a stateless *local* rule each drone
  runs on its own observed neighbours — no central solver, matching
  `ruv-drone`'s consensus/gossip ethos. `step_fleet` is an all-to-all
  convenience for offline what-if analysis only.
- **Standalone.** No path-dependency back into `ruview-swarm`; it uses its own
  light `Vec3`. The parent maps `Position3D`/`Velocity3D` at the call site — see
  ADR-172 §Integration.
- **Analytic and cheap.** The energetics are closed-form and unit-consistent
  (everything per-kilogram), cheap enough to evaluate inside the control loop.

## Scope

Part of an **industrial / civilian cooperative-UAV** project. Both behaviors are
cooperative coverage and station-keeping primitives — the bloom aggregates over
a cooperative interest map (SAR victim probability, inspection interest), exactly
like the existing coverage/allocation modules. It implements **no** adaptive
threat/target response, target acquisition, tracking-to-engage, or weapons
integration. See the repository [`NOTICE`](../../NOTICE).

## Build & test

```bash
cargo test  -p ruv-jellyfish
cargo clippy -p ruv-jellyfish --all-targets
```

Std-only, `serde` on by default (derives on the public parameter/state types so
jellyfish tuning can live in mission TOML). Apache-2.0.
