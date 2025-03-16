//! Lifecycle management for Prometheus components

use std::any::Any;

/// Trait for components that follow a lifecycle pattern
pub trait LifecycleNode: Send + Sync {
    /// Configure the node
    fn on_configure(&mut self) -> Result<(), String>;

    /// Activate the node
    fn on_activate(&mut self) -> Result<(), String>;

    /// Deactivate the node
    fn on_deactivate(&mut self) -> Result<(), String>;

    /// Clean up the node
    fn on_cleanup(&mut self) -> Result<(), String>;

    /// Convert to Any for downcasting
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// Base implementation for lifecycle nodes
pub struct LifecycleNodeBase {
    pub name: String,
    state: State,
}

/// State of a lifecycle node
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

impl LifecycleNodeBase {
    /// Create a new lifecycle node base
    pub fn new(name: &str) -> Self {
        LifecycleNodeBase {
            name: name.to_string(),
            state: State::Unconfigured,
        }
    }

    /// Get the current state
    pub fn get_state(&self) -> State {
        self.state
    }

    /// Set the state
    pub fn set_state(&mut self, state: State) {
        self.state = state;
    }
}
