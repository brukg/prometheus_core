//! Local controller implementations.
//!
//! This module contains Regulated Pure Pursuit (RPP) and will grow to include
//! MPPI, DWB, etc. Each controller implements the `Controller` trait.

pub mod rpp;

pub use rpp::RegulatedPurePursuit;
