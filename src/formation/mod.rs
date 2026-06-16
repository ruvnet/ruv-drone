//! Formation control: virtual structure, leader-follower, Reynolds flocking.
//!
//! Industrial cooperative-UAV formation **keeping** — collision-avoidant relative
//! positioning for survey/inspection/SAR fleets. This is cooperative formation and
//! collision avoidance, NOT adaptive behavior in response to threats or mission
//! objectives, and is not military "swarming". See `NOTICE`.

pub mod virtual_structure;
pub mod leader_follower;
pub mod reynolds;

pub use virtual_structure::VirtualStructure;
pub use leader_follower::LeaderFollower;
pub use reynolds::ReynoldsParams;
