# Initial validation record

Source tested: `77a65171e4198fe8aba8aeb50c174ec697c7c9d7`.

## ruOS deployment

Observed on the dedicated "ruv-drone Rust Navigation Lab" desktop on
2026-09-16 at approximately 16:03 UTC. Four exposed CPU cores, 7.8 GiB RAM,
Rust/Cargo 1.75.0, release build. Graphics reported Mesa llvmpipe software
rendering; the viewer uses Canvas 2D projected 3D geometry.

| Check | Observed result |
| --- | --- |
| Unit tests on ruOS | 12 passed |
| Seeded scenarios | 100 of 100 expected outcomes |
| Completed inspection missions | 20 |
| Expected safety holds | 80 |
| Detected collisions | 0 |
| Geofence violations | 0 |
| Dynamics violations | 0 |
| Planning p50 | 393 microseconds |
| Planning p95 | 552 microseconds |
| Planning maximum | 655 microseconds |

These are one release-build evaluation's timings, not repeated benchmark trials
or real-time guarantees. Geometry has only seven or eight boxes in a fixed
32 × 24 × 12 m volume. Results do not generalize to dense point clouds, sensor
pipelines, urban-scale maps, moving obstacles, or physical aircraft.

Browser inspection on the ruOS desktop confirmed the inspection arrival,
blocked-corridor stationary hold, and stale-sensor braking followed by hold.
The stale scenario displayed zero terminal speed and the expired evidence age.
Scenario selection produced visibly different geometry/traces and terminal
states. Orbit/top, replay, timeline, and trace download controls are supplied;
only controls explicitly exercised should be represented as manually verified.

## Additional development checks

* Unit suite passed on local Rust 1.75.0 and stable 1.98.1.
* Clippy with all targets and warnings denied passed on stable.
* JavaScript syntax check passed.
* HTTP integration exercised all five scenarios and rejected cross-origin,
  invalid Host, traversal, non-GET, and invalid-seed requests.
* The separate cloud browser could not reach the local development preview;
  visual inspection was performed through the requested ruOS connector.

Full machine-produced evaluation is retained on the development desktop in
`tools/navigation-lab/runs/ruos-evaluation.json`. CI regenerates and uploads its
own evaluation on Rust 1.75.0 and stable. Wall-clock timing fields vary, while
the seeded routes, positions, velocities, and outcomes are deterministic.

No flight readiness, upstream equivalence, or strict clean-room claim is made.
