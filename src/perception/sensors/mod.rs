//! Sensor interfaces for Prometheus robot

/// A generic sensor interface
pub trait Sensor {
    /// Get the sensor name
    fn name(&self) -> &str;

    /// Update the sensor data
    fn update(&mut self) -> Result<(), String>;
}
