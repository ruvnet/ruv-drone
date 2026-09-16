# Rust Navigation Lab

A dependency-free Rust navigation simulator with an interactive browser visualizer.
This is a civilian research prototype, not an aircraft controller or an equivalent
replacement for a PX4/ROS 2 stack. It has no flight authority.

## Run on the ruOS development desktop

```sh
cd tools/navigation-lab
cargo test --locked --offline
cargo run --release --locked --offline -- serve
```

Open <http://127.0.0.1:8777> on that desktop. Select a scenario and seed, generate
the trace, drag to orbit, scroll to zoom, scrub the timeline, or download JSON.
The viewer renders Rust-produced positions and velocities. It does not implement
a second planner in JavaScript. The route is drawn as an x-ray overlay so it
remains visible behind structures. Rendering is capped at 30 Hz and pixel ratio
1.5 for software-rendered remote desktops, and pauses when the tab is hidden.

The server accepts only loopback connections, validates Host and Origin, limits
headers and read time, serves an exact embedded file allowlist, and exposes only
read-only simulation endpoints. No external CDN, credentials, file browsing,
shell execution, autopilot commands, or CORS access. This tiny development HTTP
server is not suitable for public deployment. One slow local client can briefly
delay others; request timeouts bound that delay.

## What is implemented

* A deterministic six-connected 3D A* search over 9,216 bounded cells.
* Continuous segment/AABB clearance checks with a 0.35 m vehicle radius and
  0.40 m extra margin, including endpoint and shortcut validation.
* Explicit no-route, invalid-endpoint, and search-budget failures. No unvalidated
  direct-route fallback. Unknown volumes are excluded from traversable space.
* Analytic trapezoidal motion, at most 3 m/s and 2 m/s². The vehicle stops at each
  route corner. Sensor expiry and link failure trigger bounded braking on the
  validated segment followed by a hold.
* Five seeded scenarios: inspection, blocked volume, stale evidence, command link
  loss, and unknown volume. Geometry varies with the seed.
* JSON traces, repeatable replay, 100-scenario evaluation, security unit tests,
  and measured planning latency. Wall-clock timings are not deterministic.

## Reproduce evidence

```sh
cargo run --release --locked --offline -- evaluate > evaluation.json
cargo run --release --locked --offline -- run inspection 42 > inspection-42.json
cargo run --release --locked --offline -- run stale 42 > stale-42.json
```

The 100 scenarios comprise 20 seeds times five scenarios. Expected outcomes are
20 arrivals and 80 safety holds, not 100 completed missions. Acceptance checks
each swept motion segment, geofence containment, velocity, acceleration, terminal
velocity, and the expected outcome. Fixed fault injection begins at 3 seconds;
sensor expiry is 600 ms, with up to one 50 ms tick of detection quantization.

The planner's p95 is measured on the host running the binary and is not a control
loop or sensor-to-actuator latency. Run optimized builds for performance evidence.
Both no-route and successful searches contribute to the aggregate percentile.

## Scope and provenance

Authored independently using standard A*, closed-slab collision intersection,
and trapezoidal velocity profiles. No upstream navigation source was copied.
The author reviewed upstream public documentation during feasibility assessment,
so this is **not represented as a legally isolated clean-room implementation**.

All geometry is synthetic and known at startup. Unknown space is represented as
an excluded box; there is no online occupancy mapping, lidar model, localization,
wind, attitude dynamics, moving obstacle model, ROS 2 bridge, PX4 SITL connection,
multi-vehicle separation, or physical aircraft validation. Sensor age and command
link failures are simulated events, not real telemetry. This lab is isolated from
the root fleet crate; it does not repair or replace the root RRT/APF planner.

See [the architecture decision](../../docs/adr/ADR-175-rust-navigation-lab.md)
for the integration and validation gates before any aircraft-facing adapter.

