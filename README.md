# ruv-drone

**Industrial cooperative-UAV fleet coordination, in Rust.**

`ruv-drone` is a coordination layer that sits *above* a per-vehicle autopilot
(PX4 / ArduPilot) and turns a set of drones into a coordinated fleet — formation
keeping, distributed consensus, cooperative task allocation, collision-avoidant
planning, and learned multi-agent navigation. It targets **civilian missions**:
search-and-rescue, infrastructure inspection, agriculture, mapping, and emergency
telecom relay.

Part of the [RuView / wifi-densepose](https://github.com/ruvnet/wifi-densepose)
ecosystem (ADR-148), with an optional WiFi-CSI sensing payload for through-structure
presence detection. Pure Rust, `async`, edge-deployable.

> **Scope — industrial / civilian.** Cooperative formation and collision avoidance,
> **not** military "swarming". This project does not implement adaptive behavior in
> response to threats or mission objectives, target acquisition/engagement, or weapons
> integration. Per the U.S. State Department's clarification distinguishing cooperative
> and formation operation from military swarming, the maintainer's assessment is that
> **USML Category VIII(h)(12) does not apply.** This is not legal advice; final export
> classification is the maintainer's / export counsel's responsibility. See [`NOTICE`](./NOTICE).

## Cooperative coordination vs. military swarming

The line this project deliberately stays on:

| Capability | `ruv-drone` (cooperative) | Military "swarming" (out of scope) |
|---|:---:|:---:|
| Relative positioning — formation keeping (virtual structure, leader-follower, flocking) | ✅ | — |
| De-confliction — collision avoidance (RRT-APF) | ✅ | — |
| Shared state — Raft consensus + gossip / mesh | ✅ | — |
| Tasking — cooperative task allocation (auction / FNN) | ✅ | — |
| Learning — MAPPO cooperative navigation | ✅ | — |
| Adaptive behavior in response to threats / mission objectives | ❌ not implemented | controlled |
| Target acquisition / tracking-to-engage / fire control | ❌ not implemented | controlled |
| Weapons or countermeasure integration | ❌ not implemented | controlled |

## Where it sits

`ruv-drone` is a **coordination layer**, not a flight controller — it complements,
rather than replaces, your autopilot and transport:

| Layer | Handled by |
|-------|------------|
| Per-vehicle flight control | PX4 / ArduPilot (via the `FlightController` trait; sim included) |
| Transport | MAVLink v2 (HMAC-SHA256 signed) / DDS |
| **Fleet coordination** | **`ruv-drone`** — consensus, formation, allocation, coverage planning |
| Sensing payload (optional) | WiFi-CSI pipeline (ESP32-S3 → edge), multi-drone fusion |

## Highlights

- **Hierarchical-mesh topology** — cluster heads over Raft consensus; inter-cluster gossip for map dissemination
- **Formation control** — virtual structure, leader-follower, Reynolds flocking
- **Collision-avoidant planning** — RRT\* with Artificial Potential Field reactive avoidance
- **3-phase area coverage** — boustrophedon sweep → Bayesian probability grid → multi-drone triangulation
- **Cooperative task allocation** — auction-based bidding with an FNN bid scorer
- **MAPPO multi-agent RL** — 64-dim local observation, CTDE training, optional INT8 (ONNX) inference; real Candle PPO under the `train` feature
- **Security hardening** — MAVLink v2 signing, UWB GPS anti-spoofing, onboard geofencing, Remote ID
- **Fail-safe state machine** — 10-state, GCS-independent onboard safety
- **Sim & training** — synthetic CSI generation, Gazebo / PX4 SITL interface, TOML mission configs

## Quick start

```rust
use wifi_densepose_swarm::{config::SwarmConfig, demo::scenario::DemoScenario};

// Load a mission profile
let config = SwarmConfig::sar_default();

// Run a demo scenario
let scenario = DemoScenario::sar_rubble_field(4); // 4-drone SAR
let estimated_secs = scenario.estimate_coverage_time_secs();
// → < 240 s for 4 drones over 400×400 m
```

```bash
cargo build                 # core coordination layer
cargo build --features full # + mavlink, onnx, demo
cargo test
```

## Mission profiles

| Profile | Drones | Area | Application |
|---------|--------|------|-------------|
| `sar` | 6–12 | 400×400 m | Structural-collapse victim search |
| `inspection` | 3–6 | Linear corridor | Infrastructure (power lines, bridges) |
| `agriculture` | 4–12 | Field-configurable | NDVI mapping, variable-rate spraying |
| `mine` | 2–4 | Tunnel | GPS-denied underground exploration |
| `relay` | 6–20 | Perimeter | Emergency telecom relay chain |
| `demo` | Any | Configurable | Synthetic CSI, configurable scenarios |

## Crate features

| Feature | Description |
|---------|-------------|
| `default` | Core types, topology/consensus, formation, allocation, planning, sensing, failsafe, config, MARL |
| `mavlink` | MAVLink v2 protocol support |
| `onnx` | ONNX Runtime backend for MARL actor inference (INT8) |
| `simulation` / `demo` | Simulation mode + synthetic-CSI scenario runners |
| `train` / `cuda` | Real Candle autodiff PPO training (GPU optional) |
| `ruflo` | Ruflo AI-agent HTTP backend integration |
| `full` | `mavlink` + `onnx` + `demo` |

## Module structure

```
src/
├── types.rs       — NodeId, DroneState, SwarmTask, SwarmError, FailSafeState
├── topology/      — Raft consensus, gossip dissemination, MeshTopology
├── formation/     — VirtualStructure, LeaderFollower, Reynolds flocking
├── planning/      — RRT-APF planner, 3-phase coverage, Bayesian grid, pheromone
├── allocation/    — auction-based task allocation, FNN bid scorer
├── sensing/       — CSI payload pipeline, multi-drone fusion, OccWorld bridge
├── marl/          — MAPPO actor, LocalObservation, reward shaping, Candle PPO
├── security/      — MAVLink signing, UWB anti-spoofing, geofencing, Remote ID
├── failsafe/      — 10-state onboard fail-safe machine
├── config/        — TOML SwarmConfig with mission presets
├── demo/          — synthetic CSI, DemoScenario runners
└── integration/   — FlightController trait (PX4 / ArduPilot / sim)
```

## Related ADRs

| ADR | Title | Relation |
|-----|-------|----------|
| ADR-148 | Drone Swarm Control System | This crate |
| ADR-147 | OccWorld Occupancy World Model | Environment prior via `sensing::occworld_bridge` |
| ADR-134 | CSI→CIR ISTA Sparse Recovery | Drone payload sensing |
| ADR-146 | RF Encoder Multitask Heads | Drone payload inference |
| ADR-016 | RuVector Training Integration | CrossViewpointAttention |

## Performance targets

Engineering targets (not yet independently benchmarked end-to-end), against the
single-drone Wi2SAR baseline:

| Metric | Wi2SAR baseline (1 drone) | 4-drone target |
|--------|--------------------------|----------------|
| Coverage | 160,000 m² | 160,000 m² |
| Time | 13.5 min | ≤ 4 min |
| Localization | 5 m | ≤ 2 m (3-view fusion) |
| MARL inference | N/A | ≤ 5 ms (INT8, release) |
| Raft election | N/A | ≤ 300 ms |

## License

Apache-2.0. See [`NOTICE`](./NOTICE) for scope and export details.
