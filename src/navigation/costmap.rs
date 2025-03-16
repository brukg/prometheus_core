//! Costmap for navigation

/// A costmap for navigation
pub struct Costmap {
    // Costmap data
    width: usize,
    height: usize,
    resolution: f64,
    data: Vec<u8>,
}

impl Costmap {
    /// Create a new costmap
    pub fn new() -> Self {
        Costmap {
            width: 100,
            height: 100,
            resolution: 0.05,
            data: vec![0; 100 * 100],
        }
    }
    
    /// Update the costmap with new sensor data
    pub fn update(&self) {
        // Update costmap based on sensor data
    }
    
    /// Get the cost at a specific position
    pub fn get_cost(&self, x: f64, y: f64) -> u8 {
        // Convert world coordinates to grid coordinates
        let grid_x = (x / self.resolution) as usize;
        let grid_y = (y / self.resolution) as usize;
        
        if grid_x < self.width && grid_y < self.height {
            self.data[grid_y * self.width + grid_x]
        } else {
            255 // Unknown/out of bounds
        }
    }
} 