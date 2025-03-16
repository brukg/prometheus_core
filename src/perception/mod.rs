//! Perception module for Prometheus robot
pub mod filters;
pub mod localization;
pub mod sensors;

use self::localization::Localizer;
use crate::lifecycle::{LifecycleNode, LifecycleNodeBase, State};
use std::any::Any;

/// Perception stack for the robot
pub struct PerceptionStack {
    base: LifecycleNodeBase,
    localizer: Localizer,
}

impl PerceptionStack {
    /// Create a new perception stack
    pub fn new() -> Self {
        PerceptionStack {
            base: LifecycleNodeBase::new("perception_stack"),
            localizer: Localizer::new(),
        }
    }

    /// Get the current pose estimate
    pub fn get_pose(&self) -> (f64, f64, f64) {
        self.localizer.get_pose()
    }
}

impl LifecycleNode for PerceptionStack {
    fn on_configure(&mut self) -> Result<(), String> {
        println!("Configuring perception stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_activate(&mut self) -> Result<(), String> {
        println!("Activating perception stack");
        self.base.set_state(State::Active);
        Ok(())
    }

    fn on_deactivate(&mut self) -> Result<(), String> {
        println!("Deactivating perception stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_cleanup(&mut self) -> Result<(), String> {
        println!("Cleaning up perception stack");
        self.base.set_state(State::Unconfigured);
        Ok(())
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
