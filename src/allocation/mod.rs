//! Task allocation: auction-based and FNN-scored bid evaluation.
//!
//! Industrial cooperative task assignment — distributing survey/coverage/inspection
//! /delivery tasks across a civilian fleet. This is cooperative operation, NOT
//! adaptive behavior in response to threats or mission objectives. See `NOTICE`.

pub mod auction;
pub mod fnn;

pub use auction::{AuctionAllocator, Bid};
pub use fnn::FnnScorer;
