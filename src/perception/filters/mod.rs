//! Filtering algorithms for sensor data

/// A generic filter interface
pub trait Filter<T> {
    /// Filter the input data
    fn filter(&self, input: T) -> T;
}

/// A simple moving average filter
pub struct MovingAverageFilter {
    window_size: usize,
    buffer: Vec<f64>,
}

impl MovingAverageFilter {
    /// Create a new moving average filter
    pub fn new(window_size: usize) -> Self {
        MovingAverageFilter {
            window_size,
            buffer: Vec::with_capacity(window_size),
        }
    }
}

impl Filter<f64> for MovingAverageFilter {
    fn filter(&self, input: f64) -> f64 {
        if self.buffer.is_empty() {
            return input;
        }

        let sum: f64 = self.buffer.iter().sum();
        (sum + input) / (self.buffer.len() as f64 + 1.0)
    }
}
