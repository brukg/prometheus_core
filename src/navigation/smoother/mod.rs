//! Path smoother implementations.
//!
//! This module contains the Simple Smoother and will grow to include
//! Savitzky-Golay and other smoother implementations.

pub mod simple;

pub use simple::SimpleSmoother;
