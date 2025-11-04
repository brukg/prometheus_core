//! Costmap for navigation
//!
//! This module provides a costmap implementation for robot navigation.
//! It supports both global and local costmaps, with functions for querying
//! costs and detecting obstacles.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Cost values for different types of cells
pub mod cost_values {
    pub const LETHAL_OBSTACLE: u8 = 254;
    pub const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;
    pub const MEDIUM_COST: u8 = 128;
    pub const LOW_COST: u8 = 50;
    pub const NO_COST: u8 = 0;
    pub const UNKNOWN_COST: u8 = 255;
}

/// Coordinate in the map
#[derive(Debug, Clone, Copy)]
pub struct MapPoint {
    pub x: i32,
    pub y: i32,
}

/// World coordinate
#[derive(Debug, Clone, Copy)]
pub struct WorldPoint {
    pub x: f64,
    pub y: f64,
}

/// A costmap for navigation
#[derive(Debug, Clone)]
pub struct Costmap {
    // Costmap metadata
    pub width: usize,
    pub height: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>,
    pub is_global: bool,
    // Parameters
    pub inflation_radius: f64,
    pub cost_scaling_factor: f64,
    pub update_frequency: f64,
    pub publish_frequency: f64,
    // State for tracking changes
    pub updated: bool,
}

impl Costmap {
    /// Create a new costmap
    pub fn new(is_global: bool) -> Self {
        let (width, height) = if is_global {
            (500, 500) // Global maps are typically larger
        } else {
            (100, 100) // Local maps are smaller
        };
        
        Costmap {
            width,
            height,
            resolution: 0.05,  // 5cm resolution
            origin_x: if is_global { -12.5 } else { -2.5 }, // Center the map
            origin_y: if is_global { -12.5 } else { -2.5 },
            data: vec![cost_values::UNKNOWN_COST; width * height],
            is_global,
            inflation_radius: 0.55,
            cost_scaling_factor: 3.0,
            update_frequency: if is_global { 1.0 } else { 5.0 },
            publish_frequency: if is_global { 1.0 } else { 2.0 },
            updated: false,
        }
    }
    
    /// Configure the costmap with parameters
    pub fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        if let Some(&resolution) = params.get("resolution") {
            if resolution <= 0.0 {
                return Err("Resolution must be positive".to_string());
            }
            self.resolution = resolution;
        }
        
        if let Some(&width) = params.get("width") {
            if width <= 0.0 {
                return Err("Width must be positive".to_string());
            }
            self.width = width as usize;
            // Resize data array
            self.data = vec![cost_values::UNKNOWN_COST; self.width * self.height];
        }
        
        if let Some(&height) = params.get("height") {
            if height <= 0.0 {
                return Err("Height must be positive".to_string());
            }
            self.height = height as usize;
            // Resize data array
            self.data = vec![cost_values::UNKNOWN_COST; self.width * self.height];
        }
        
        if let Some(&origin_x) = params.get("origin_x") {
            self.origin_x = origin_x;
        }
        
        if let Some(&origin_y) = params.get("origin_y") {
            self.origin_y = origin_y;
        }
        
        if let Some(&inflation_radius) = params.get("inflation_radius") {
            if inflation_radius < 0.0 {
                return Err("Inflation radius must be non-negative".to_string());
            }
            self.inflation_radius = inflation_radius;
        }
        
        if let Some(&cost_scaling_factor) = params.get("cost_scaling_factor") {
            if cost_scaling_factor <= 0.0 {
                return Err("Cost scaling factor must be positive".to_string());
            }
            self.cost_scaling_factor = cost_scaling_factor;
        }
        
        if let Some(&update_frequency) = params.get("update_frequency") {
            if update_frequency <= 0.0 {
                return Err("Update frequency must be positive".to_string());
            }
            self.update_frequency = update_frequency;
        }
        
        if let Some(&publish_frequency) = params.get("publish_frequency") {
            if publish_frequency <= 0.0 {
                return Err("Publish frequency must be positive".to_string());
            }
            self.publish_frequency = publish_frequency;
        }
        
        Ok(())
    }
    
    /// Update the costmap with new map data (for global costmap)
    pub fn update_from_map(&mut self, map_data: &[u8], width: usize, height: usize, resolution: f64, 
                          origin_x: f64, origin_y: f64) -> Result<(), String> {
        if width != self.width || height != self.height {
            return Err(format!("Map dimensions do not match costmap dimensions: map is {}x{}, costmap is {}x{}",
                              width, height, self.width, self.height));
        }
        
        self.resolution = resolution;
        self.origin_x = origin_x;
        self.origin_y = origin_y;
        
        // Copy the map data
        for i in 0..self.data.len() {
            let value = map_data[i];
            // Convert from occupancy grid values (0-100) to cost values
            self.data[i] = match value {
                0 => cost_values::NO_COST,                   // Free space
                100 => cost_values::LETHAL_OBSTACLE,         // Occupied
                255 => cost_values::UNKNOWN_COST,            // Unknown
                _ => ((value as f64 / 100.0) * 253.0) as u8, // Partially occupied
            };
        }
        
        // Mark the costmap as updated
        self.updated = true;
        
        // Inflate obstacles
        self.inflate_obstacles();
        
        Ok(())
    }
    
    /// Update the costmap with new sensor data (for local costmap)
    pub fn update_from_sensor(&mut self, sensor_data: &[WorldPoint], sensor_range: f64) {
        // Clear the previous sensor data (for local costmaps)
        if !self.is_global {
            // Only reset to unknown for unknown cells, keep obstacles from the map
            for i in 0..self.data.len() {
                if self.data[i] != cost_values::LETHAL_OBSTACLE && 
                   self.data[i] != cost_values::INSCRIBED_INFLATED_OBSTACLE {
                    self.data[i] = cost_values::NO_COST;
                }
            }
        }
        
        // Mark cells as obstacles based on sensor data
        for point in sensor_data {
            let (grid_x, grid_y) = self.world_to_map(point.x, point.y);
            if grid_x >= 0 && grid_x < self.width as i32 && 
               grid_y >= 0 && grid_y < self.height as i32 {
                let index = (grid_y as usize) * self.width + (grid_x as usize);
                self.data[index] = cost_values::LETHAL_OBSTACLE;
            }
        }
        
        // Inflate obstacles
        self.inflate_obstacles();
        
        // Mark the costmap as updated
        self.updated = true;
    }
    
    /// Get the cost at a specific position in world coordinates
    pub fn get_cost(&self, x: f64, y: f64) -> u8 {
        let (grid_x, grid_y) = self.world_to_map(x, y);
        self.get_cost_map(grid_x, grid_y)
    }
    
    /// Get the cost at a specific position in map coordinates
    pub fn get_cost_map(&self, grid_x: i32, grid_y: i32) -> u8 {
        if grid_x >= 0 && grid_x < self.width as i32 && 
           grid_y >= 0 && grid_y < self.height as i32 {
            let index = (grid_y as usize) * self.width + (grid_x as usize);
            self.data[index]
        } else {
            cost_values::UNKNOWN_COST // Unknown/out of bounds
        }
    }
    
    /// Check if a point is an obstacle
    pub fn is_obstacle(&self, x: f64, y: f64) -> bool {
        let cost = self.get_cost(x, y);
        cost >= cost_values::INSCRIBED_INFLATED_OBSTACLE
    }
    
    /// Check if a line between two points crosses an obstacle
    pub fn line_cost(&self, x1: f64, y1: f64, x2: f64, y2: f64) -> u8 {
        let dx = x2 - x1;
        let dy = y2 - y1;
        let distance = (dx * dx + dy * dy).sqrt();
        let steps = (distance / (self.resolution * 0.5)).ceil() as i32;
        
        let mut max_cost = cost_values::NO_COST;
        
        for i in 0..=steps {
            let t = if steps > 0 { i as f64 / steps as f64 } else { 0.0 };
            let x = x1 + t * dx;
            let y = y1 + t * dy;
            let cost = self.get_cost(x, y);
            if cost > max_cost {
                max_cost = cost;
            }
            
            // Early exit if we hit a lethal obstacle
            if max_cost >= cost_values::LETHAL_OBSTACLE {
                return max_cost;
            }
        }
        
        max_cost
    }
    
    /// Convert world coordinates to map coordinates
    pub fn world_to_map(&self, x: f64, y: f64) -> (i32, i32) {
        let grid_x = ((x - self.origin_x) / self.resolution).floor() as i32;
        let grid_y = ((y - self.origin_y) / self.resolution).floor() as i32;
        (grid_x, grid_y)
    }
    
    /// Convert map coordinates to world coordinates (cell center)
    pub fn map_to_world(&self, grid_x: i32, grid_y: i32) -> (f64, f64) {
        let x = self.origin_x + (grid_x as f64 + 0.5) * self.resolution;
        let y = self.origin_y + (grid_y as f64 + 0.5) * self.resolution;
        (x, y)
    }
    
    /// Inflate obstacles in the costmap
    fn inflate_obstacles(&mut self) {
        // Create a temporary buffer for the inflated map
        let mut inflated_data = vec![cost_values::NO_COST; self.width * self.height];
        
        // Calculate the number of cells for inflation
        let cell_inflation_radius = (self.inflation_radius / self.resolution).ceil() as i32;
        
        // For each cell in the costmap
        for y in 0..self.height {
            for x in 0..self.width {
                let index = y * self.width + x;
                
                // If this cell is an obstacle, inflate it
                if self.data[index] == cost_values::LETHAL_OBSTACLE {
                    inflated_data[index] = cost_values::LETHAL_OBSTACLE;
                    
                    // Inflate around the obstacle
                    for dy in -cell_inflation_radius..=cell_inflation_radius {
                        for dx in -cell_inflation_radius..=cell_inflation_radius {
                            let nx = x as i32 + dx;
                            let ny = y as i32 + dy;
                            
                            // Skip if out of bounds
                            if nx < 0 || nx >= self.width as i32 || ny < 0 || ny >= self.height as i32 {
                                continue;
                            }
                            
                            // Calculate the distance to the obstacle
                            let distance = ((dx * dx + dy * dy) as f64).sqrt() * self.resolution;
                            
                            // If within inflation radius
                            if distance <= self.inflation_radius {
                                let new_index = ny as usize * self.width + nx as usize;
                                
                                // Calculate cost based on distance
                                let cost = if distance < 0.05 {
                                    // Very close, mark as inscribed
                                    cost_values::INSCRIBED_INFLATED_OBSTACLE
                                } else {
                                    // Exponential decay based on distance
                                    let factor = (-self.cost_scaling_factor * (distance / self.inflation_radius)).exp();
                                    let cost = (cost_values::INSCRIBED_INFLATED_OBSTACLE as f64 * factor) as u8;
                                    std::cmp::max(cost, cost_values::LOW_COST)
                                };
                                
                                // Update if the new cost is higher
                                if cost > inflated_data[new_index] {
                                    inflated_data[new_index] = cost;
                                }
                            }
                        }
                    }
                } else if self.data[index] == cost_values::UNKNOWN_COST && inflated_data[index] == cost_values::NO_COST {
                    // Preserve unknown cells
                    inflated_data[index] = cost_values::UNKNOWN_COST;
                }
            }
        }
        
        // Update the costmap data with the inflated values
        self.data = inflated_data;
    }
    
    /// Check if a path is collision-free
    pub fn is_path_valid(&self, path: &[(f64, f64)]) -> bool {
        if path.len() < 2 {
            return true;
        }
        
        for i in 0..path.len() - 1 {
            let cost = self.line_cost(path[i].0, path[i].1, path[i+1].0, path[i+1].1);
            if cost >= cost_values::INSCRIBED_INFLATED_OBSTACLE {
                return false; // Collision detected
            }
        }
        
        true // No collisions
    }
}

/// CostmapManager manages both global and local costmaps
#[derive(Debug)]
pub struct CostmapManager {
    global_costmap: Arc<RwLock<Costmap>>,
    local_costmap: Arc<RwLock<Costmap>>,
}

impl CostmapManager {
    /// Create a new costmap manager
    pub fn new() -> Self {
        CostmapManager {
            global_costmap: Arc::new(RwLock::new(Costmap::new(true))),
            local_costmap: Arc::new(RwLock::new(Costmap::new(false))),
        }
    }
    
    /// Get a reference to the global costmap
    pub fn global_costmap(&self) -> Arc<RwLock<Costmap>> {
        Arc::clone(&self.global_costmap)
    }
    
    /// Get a reference to the local costmap
    pub fn local_costmap(&self) -> Arc<RwLock<Costmap>> {
        Arc::clone(&self.local_costmap)
    }
    
    /// Configure the costmaps
    pub fn configure(&self, global_params: &HashMap<String, f64>, local_params: &HashMap<String, f64>) -> Result<(), String> {
        let mut global = self.global_costmap.write().map_err(|_| "Failed to lock global costmap".to_string())?;
        let mut local = self.local_costmap.write().map_err(|_| "Failed to lock local costmap".to_string())?;
        
        global.configure(global_params)?;
        local.configure(local_params)?;
        
        Ok(())
    }
    
    /// Update the global costmap from a map
    pub fn update_global_map(&self, map_data: &[u8], width: usize, height: usize, resolution: f64, 
                            origin_x: f64, origin_y: f64) -> Result<(), String> {
        let mut global = self.global_costmap.write().map_err(|_| "Failed to lock global costmap".to_string())?;
        global.update_from_map(map_data, width, height, resolution, origin_x, origin_y)
    }
    
    /// Update the local costmap from sensor data
    pub fn update_local_map(&self, sensor_data: &[WorldPoint], sensor_range: f64) -> Result<(), String> {
        let mut local = self.local_costmap.write().map_err(|_| "Failed to lock local costmap".to_string())?;
        local.update_from_sensor(sensor_data, sensor_range);
        Ok(())
    }
    
    /// Check if a point is an obstacle in either costmap
    pub fn is_obstacle(&self, x: f64, y: f64) -> Result<bool, String> {
        let global = self.global_costmap.read().map_err(|_| "Failed to lock global costmap".to_string())?;
        let local = self.local_costmap.read().map_err(|_| "Failed to lock local costmap".to_string())?;
        
        // Check both costmaps, prioritizing the local one
        Ok(local.is_obstacle(x, y) || global.is_obstacle(x, y))
    }
    
    /// Check if a path is valid (collision-free) using both costmaps
    pub fn is_path_valid(&self, path: &[(f64, f64)]) -> Result<bool, String> {
        let global = self.global_costmap.read().map_err(|_| "Failed to lock global costmap".to_string())?;
        let local = self.local_costmap.read().map_err(|_| "Failed to lock local costmap".to_string())?;
        
        // Check in both costmaps
        Ok(global.is_path_valid(path) && local.is_path_valid(path))
    }
} 