# ADR-175: Independently implemented Rust navigation simulation lab

Status: Proposed

## Context

The fleet project needs reproducible evidence for per-vehicle navigation before
adopting a real navigation backend. Its current simulated flight controller
immediately applies positions. Its RRT/APF implementation has an unchecked direct
fallback and does not continuously validate segments. A convincing animation
alone would not resolve these limitations.

The external PX4/ROS 2 navigation project informed feasibility discussions. Its
public documentation was reviewed, so this implementation must not be described
as a strictly isolated clean-room effort. No code from that project is included.

## Decision

Add an isolated, dependency-free crate in `tools/navigation-lab`. It builds with
Rust 1.75 or newer without resolving the fleet crate's optional GPU and git
dependencies. Maintain the root package's runtime and API unchanged in this PR.

Inputs are a typed scenario and a u32 seed. Outputs are a bounded route, a Rust
kinematic trace, terminal outcome, validation counts, and measured search time.
Assumptions are exact localization, known synthetic geometry, one vehicle,
instant fault detection at the next simulation tick, and constant braking ability.

Use deterministic A*, conservative expanded-box segment checks, and rest-to-rest
trapezoidal segments. Bound search at 9,216 expansions. Unknown volumes block
routes. Every failure is explicit, with no unchecked straight-line fallback.
The JavaScript visualizer consumes traces; it never supplies flight commands or
reimplements motion. A local HTTP binary embeds assets and prohibits arbitrary
file access, command execution, cross-origin access, and non-local Host headers.

The initial renderer uses projected 3D geometry on Canvas 2D to work on the ruOS
desktop's software renderer without WebGL, a GPU, downloads, or npm dependencies.

## Acceptance evidence

1. Unit tests cover thin barriers, tangent collision, nonfinite inputs, budget
   exhaustion, no-route failure, unknown volumes, deterministic replay, and braking.
2. Twenty seeds across five scenarios must yield 20 arrivals and 80 expected holds,
   zero detected collisions/geofence/dynamics violations, and zero terminal speed.
3. The browser must display the Rust trace, allow replay and scenario selection,
   and show an explicit unavailable state if the engine cannot be reached.
4. Record release-build planning p50/p95/max with host details. Do not conflate
   this with sensor-to-actuator timing, real-time guarantees, or flight readiness.

## Subsequent gates

* Fix the root planner's unchecked fallback and continuously validate its edges;
  cover all callers' handling of no-route before changing its contract.
* Introduce a reviewed navigation-provider interface, coordinate frame contracts,
  bounded timestamps, mission receipts, and one owner of PX4 setpoint publication.
* Add online occupancy evidence, sensor/pose uncertainty, and dynamic-obstacle
  stopping margins. Faults must cover localization failures and actuator lag.
* Validate PX4 SITL, then hardware-in-the-loop and device-specific resource budgets.
* Preserve LatentMesh and forecasting as advisory inputs without arming authority.

Aircraft integration remains a separate reviewed change. Passing this lab's tests
does not validate flight safety or establish equivalence with an upstream system.

