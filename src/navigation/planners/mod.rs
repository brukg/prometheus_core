//! Global planner implementations.
//!
//! This module contains NavFn (Dijkstra/A*) and will grow to include SMAC planners,
//! Theta*, etc. Each planner implements the `GlobalPlanner` trait.

pub mod navfn;

pub use navfn::NavFnPlanner;
