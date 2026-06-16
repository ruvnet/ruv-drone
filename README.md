# ruv-drone (wifi-densepose-swarm)

> ⚠️ **Export-controlled capability.** Swarming coordination features (formation
> control, Raft consensus, task allocation) are gated behind the `itar-unrestricted`
> feature flag per **USML Category VIII(h)(12)**. Default builds compile only safe
> stubs. **Do not enable `itar-unrestricted` for international distribution without
> export counsel review.** See [`NOTICE`](./NOTICE).

Drone swarm control system for the RuView wifi-densepose workspace. Implements ADR-148.

## Overview

`wifi-densepose-swarm` provides a hierarchical-mesh drone swarm coordination system
with Raft consensus, MAPPO-based multi-agent reinforcement learning, and tight
integration with the existing WiFi CSI sensing pipeline (`wifi-densepose-signal`,
`wifi-densepose-ruvector`).

## Features

- **Hierarchical-Mesh Topology** — cluster heads over Raft consensus; inter-cluster Gossip for map dissemination
- **Formation Control** — F1 VirtualStructure, F2 LeaderFollower, F3 Reynolds flocking
- **3-Phase Coverage** — boustrophedon sweep → Bayesian probability grid → multi-drone triangulation
- **RRT-APF Path Planner** — RRT* with Artificial Potential Field reactive collision avoidance
- **MARL Actor (MAPPO)** — 64-dim local observation, 3-layer MLP actor, CTDE training interface
- **CSI Sensing Integration** — drone payload pipeline (ESP32-S3 → Jetson), multi-drone CSI fusion
- **OccWorld Bridge** — integrates ADR-147 OccWorld occupancy prior as path planner environment
- **Security Hardening** — MAVLink v2 HMAC-SHA256 signing, UWB GPS anti-spoofing, onboard geofencing, Remote ID
- **Fail-Safe State Machine** — 10-state onboard safety system, GCS-independent
- **Demo & Training Modes** — synthetic CSI generation, Gazebo/PX4 SITL interface, TOML mission configs

## ITAR Notice

> ⚠️ **Export-controlled capability.** Swarming coordination features (formation control,
> Raft consensus, task allocation) are gated behind the `itar-unrestricted` feature flag
> per **USML Category VIII(h)(12)**. Default builds compile only safe stubs.
> Do not enable `itar-unrestricted` for international distribution without export counsel review.

## Crate Features

| Feature | Description |
|---------|-------------|
| `default` | Core types, sensing, failsafe, config, MARL — no ITAR-gated code |
| `itar-unrestricted` | Enables formation control, Raft consensus, task allocation |
| `mavlink` | MAVLink v2 protocol support |
| `onnx` | ONNX Runtime backend for MARL actor inference (INT8) |
| `simulation` | Simulation-mode stubs |
| `demo` | Synthetic CSI generation, scenario runners |
| `full` | All of the above |

## Quick Start

```rust
use wifi_densepose_swarm::{config::SwarmConfig, demo::scenario::DemoScenario};

// Load a mission profile
let config = SwarmConfig::sar_default();

// Run a demo scenario
let scenario = DemoScenario::sar_rubble_field(4); // 4-drone SAR
let estimated_secs = scenario.estimate_coverage_time_secs();
// → < 240 s for 4 drones over 400×400 m (beyond Wi2SAR SOTA single-drone baseline)
```

## Mission Profiles

| Profile | Drones | Area | Application |
|---------|--------|------|-------------|
| `sar` | 6–12 | 400×400 m | Structural collapse victim search |
| `inspection` | 3–6 | Linear corridor | Infrastructure (power lines, bridges) |
| `agriculture` | 4–12 | Field-configurable | NDVI mapping, variable-rate spraying |
| `mine` | 2–4 | Tunnel | GPS-denied underground exploration |
| `relay` | 6–20 | Perimeter | Emergency telecom relay chain |
| `demo` | Any | Configurable | Synthetic CSI, configurable victims |

## Module Structure

```
src/
├── types.rs            — NodeId, DroneState, SwarmTask, SwarmError, FailSafeState
├── topology/           — Raft consensus¹, Gossip dissemination, MeshTopology
├── formation/          — VirtualStructure¹, LeaderFollower¹, Reynolds flocking¹
├── planning/           — RRT-APF planner, 3-phase coverage, Bayesian grid, pheromone
├── allocation/         — Auction-based task allocation¹, FNN bid scorer¹
├── sensing/            — CSI payload pipeline, multi-drone fusion, OccWorld bridge
├── marl/               — MAPPO actor, LocalObservation, reward shaping, TrainingConfig
├── security/           — MAVLink signing, UWB anti-spoofing, geofencing, Remote ID
├── failsafe/           — 10-state onboard fail-safe machine
├── config/             — TOML SwarmConfig with mission presets
├── demo/               — Synthetic CSI, DemoScenario runners
├── integration/        — FlightController trait (PX4/ArduPilot/Sim)
└── bench_support.rs    — Criterion fixture generators

¹ Requires `itar-unrestricted` feature.
```

## Related ADRs

| ADR | Title | Relation |
|-----|-------|----------|
| ADR-148 | Drone Swarm Control System | This crate |
| ADR-147 | OccWorld Occupancy World Model | Environment prior via `sensing::occworld_bridge` |
| ADR-134 | CSI→CIR ISTA Sparse Recovery | Drone payload sensing |
| ADR-146 | RF Encoder Multitask Heads | Drone payload inference |
| ADR-016 | RuVector Training Integration | CrossViewpointAttention |

## Performance Targets (vs. Wi2SAR SOTA)

| Metric | Wi2SAR baseline (1 drone) | 4-drone target |
|--------|--------------------------|----------------|
| Coverage | 160,000 m² | 160,000 m² |
| Time | 13.5 min | ≤ 4 min |
| Localization | 5 m | ≤ 2 m (3-view fusion) |
| MARL inference | N/A | ≤ 5 ms (INT8, release) |
| Raft election | N/A | ≤ 300 ms |
