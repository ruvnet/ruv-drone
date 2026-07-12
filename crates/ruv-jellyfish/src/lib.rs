//! # ruv-jellyfish — jellyfish-inspired swarm behaviors (ADR-172)
//!
//! Energy-efficient loiter and aggregation primitives for cooperative UAV
//! fleets, sitting alongside the geometry-first behaviors in the parent
//! `ruview-swarm` crate (`formation`, `planning`, `allocation`). Where those
//! optimize *shapes* and *paths*, `ruv-jellyfish` optimizes *endurance* and
//! *presence* for the missions that are battery-bound rather than speed-bound:
//! relay chains, SAR re-scan loops, persistent agricultural monitoring.
//!
//! Two ideas, both drawn from real jellyfish biomechanics and ecology:
//!
//! * **Pulse-and-drift gait** ([`pulse`], [`energy`]) — the lowest-cost-of-
//!   transport gait measured in any swimmer: a powered pulse, a free
//!   vortex-recapture bonus, then a passive coast. Modelled analytically and as
//!   a real-time state machine.
//! * **Bloom aggregation** ([`bloom`], [`field`]) — a decentralized local rule
//!   that densifies a "smack" over high-value regions and relaxes to broad
//!   coverage where the field is flat, riding the wind/current field to keep
//!   station cheaply.
//!
//! [`controller::JellyfishController`] unifies both around one energy budget.
//!
//! ## Scope
//!
//! This crate is part of an **industrial / civilian cooperative-UAV** project.
//! Both behaviors are cooperative coverage and station-keeping primitives — the
//! bloom aggregates over a cooperative interest map (SAR victim probability,
//! inspection interest), exactly like the existing coverage/allocation modules.
//! It implements **no** adaptive threat/target response, target acquisition,
//! tracking-to-engage, or weapons integration. See the repository `NOTICE`.
//!
//! ## Quick start
//!
//! ```
//! use ruv_jellyfish::{
//!     JellyfishController,
//!     field::{GaussianHotspot, HotspotField, UniformFlow},
//!     vec3::Vec3,
//! };
//!
//! // A SAR crew loitering over a high-probability cell in a light breeze.
//! let value = HotspotField::new(vec![GaussianHotspot {
//!     centre: Vec3::new(120.0, 80.0, 0.0),
//!     peak: 1.0,
//!     sigma: 35.0,
//! }]);
//! let wind = UniformFlow(Vec3::new(1.0, 0.5, 0.0));
//!
//! let mut drone = JellyfishController::with_budget(500_000.0); // J/kg
//! let neighbours = [Vec3::new(130.0, 90.0, 0.0)];
//! let step = drone.loiter(Vec3::new(100.0, 70.0, 0.0), &neighbours, &value, &wind, 0.0, 0.1);
//!
//! assert!(step.command.aggregation >= 0.0 && step.command.aggregation < 1.0);
//! assert!(drone.budget_remaining() < 500_000.0);
//! ```

pub mod bloom;
pub mod controller;
pub mod energy;
pub mod field;
pub mod pulse;
pub mod vec3;

pub use bloom::{BloomCommand, BloomController, BloomParams};
pub use controller::{JellyfishController, LoiterStep};
pub use energy::{EnergyModel, Gait};
pub use field::{
    ConvergentFlow, FlowField, GaussianHotspot, HotspotField, NoFlow, UniformFlow, ValueField,
};
pub use pulse::{GaitParams, GaitTelemetry, Phase, PulseDriftGait};
pub use vec3::Vec3;
