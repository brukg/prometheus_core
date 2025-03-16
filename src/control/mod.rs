//! Control module for Prometheus robot
pub mod controllers;
pub mod trajectory;

use self::controllers::DifferentialDriveController;
use crate::lifecycle::{LifecycleNode, LifecycleNodeBase, State};
use std::any::Any;

/// Control stack for the robot
pub struct ControlStack {
    base: LifecycleNodeBase,
    diff_drive_controller: DifferentialDriveController,
}

impl ControlStack {
    /// Create a new control stack
    pub fn new() -> Self {
        ControlStack {
            base: LifecycleNodeBase::new("control_stack"),
            diff_drive_controller: DifferentialDriveController::new(),
        }
    }

    /// Compute control commands
    pub fn compute_velocity(
        &self,
        current_pose: (f64, f64, f64),
        target_pose: (f64, f64, f64),
    ) -> (f64, f64) {
        self.diff_drive_controller
            .compute_velocity(current_pose, target_pose)
    }
}

impl LifecycleNode for ControlStack {
    fn on_configure(&mut self) -> Result<(), String> {
        println!("Configuring control stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_activate(&mut self) -> Result<(), String> {
        println!("Activating control stack");
        self.base.set_state(State::Active);
        Ok(())
    }

    fn on_deactivate(&mut self) -> Result<(), String> {
        println!("Deactivating control stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_cleanup(&mut self) -> Result<(), String> {
        println!("Cleaning up control stack");
        self.base.set_state(State::Unconfigured);
        Ok(())
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
