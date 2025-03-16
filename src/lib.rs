pub mod behaviors;
pub mod common;
pub mod control;
pub mod lifecycle;
pub mod navigation;
pub mod perception;

use crate::lifecycle::LifecycleNode;
use crate::navigation::NavigationStack;

/// Core functionality for the Prometheus robot
pub struct PrometheusCore {
    components: Vec<Box<dyn LifecycleNode>>,
}

impl PrometheusCore {
    /// Create a new instance of PrometheusCore
    pub fn new() -> Self {
        PrometheusCore {
            components: Vec::new(),
        }
    }

    /// Register a component with the core
    pub fn register<T: LifecycleNode + 'static>(&mut self, component: T) {
        self.components.push(Box::new(component));
    }

    /// Initialize all registered components
    pub fn init(&mut self) -> Result<(), String> {
        for component in &mut self.components {
            component.on_configure()?;
            component.on_activate()?;
        }
        Ok(())
    }

    /// Shutdown all registered components
    pub fn shutdown(&mut self) -> Result<(), String> {
        for component in &mut self.components {
            component.on_deactivate()?;
            component.on_cleanup()?;
        }
        Ok(())
    }

    /// Get a reference to the navigation stack
    pub fn navigation_stack_mut(&mut self) -> Option<&mut NavigationStack> {
        // Assuming components are stored in a vector or map
        // This is a simplified example - adjust based on actual implementation
        self.components
            .iter_mut()
            .find_map(|component| component.as_any_mut().downcast_mut::<NavigationStack>())
    }
}
