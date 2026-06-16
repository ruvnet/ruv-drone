# ruvdrone

AI agent harness for ruv-drone — cooperative civilian UAV fleet coordination in Rust. All 9 hosts.

> Advanced Coding harness · domain: `software-engineering`. Generated with [create-agent-harness](https://github.com/ruvnet/agent-harness-generator).

## Project

`ruv-drone` is a Rust coordination layer above per-vehicle autopilots (PX4 / ArduPilot)
that turns drones into a cooperative fleet: formation keeping, Raft consensus, cooperative
task allocation, RRT-APF collision avoidance, and MAPPO/MARL navigation. Key modules:
`formation`, `allocation`, `planning`, `marl`, `orchestrator`, `topology`, `failsafe`,
`security` (HMAC/SHA-2), `sensing` (WiFi-CSI), `integration` (MAVLink).

Build/test: `cargo build` · `cargo test` · `cargo clippy --all-targets` · `cargo bench`.

## Behavioral rules

- **Civilian / industrial scope only (hard rule).** ruv-drone is cooperative-fleet
  coordination for SAR, inspection, agriculture, mapping, telecom relay. **Never** add or
  assist with threat-adaptive behavior, target acquisition / tracking-to-engage, fire
  control, weapons/munitions integration, or autonomous engagement. If a request implies
  any of these, stop and surface it. See the repo `NOTICE`.
- Rust idioms: keep `unsafe` out (the crate forbids it in most subsystems), follow the
  existing `async`/`tokio` patterns, run `cargo clippy` before proposing a diff.
- Use the harness's MCP tools (`mcp__ruvdrone__*`) for orchestration.
- Memory and routing are handled by the kernel — you don't need to learn them.
- Defer destructive operations to the user.

## Agents

| Agent | Tier | Role |
|---|---|---|
| `architect` | opus | Designs the change before code is written. |
| `implementer` | sonnet | Writes code that matches the surrounding style. |
| `reviewer` | opus | Hunts correctness bugs in the diff. |
| `test-writer` | sonnet | Adds the missing tests for the change. |
## Skills

- `/plan-change` — Turn a feature request into a minimal, file-level implementation plan before any code.

## Commands

- `doctor` — Health-check the harness: kernel load, MCP wiring, memory backend, host adapter.
- `review-diff` — Review the current working diff for correctness, security, and reuse.

## Architecture

This harness uses [@metaharness/kernel](https://www.npmjs.com/package/@metaharness/kernel) — a Rust-compiled WASM module with a NAPI-RS native fallback — so the same code runs identically on every platform.
