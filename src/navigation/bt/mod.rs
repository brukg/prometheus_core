//! Behavior Tree engine for Nav2-compatible navigation.
//!
//! This module implements a behavior tree engine with the standard node types
//! (Sequence, Fallback, Action, Condition, Decorator) plus Nav2-specific
//! control nodes (RecoveryNode, PipelineSequence, RoundRobin).

pub mod engine;

pub use engine::*;
