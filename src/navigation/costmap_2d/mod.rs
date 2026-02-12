//! Layered Costmap2D module — Nav2-compatible layered costmap architecture.
//!
//! This module refactors the monolithic `Costmap` into a layered architecture where
//! each layer independently updates bounds and costs on a master grid. The existing
//! `Costmap` struct is preserved for backwards compatibility and used as the master grid.

pub mod inflation_layer;
pub mod static_layer;

use crate::navigation::costmap::Costmap;
use crate::navigation::traits::CostmapLayer;
use crate::navigation::types::ParamMap;
use std::sync::{Arc, RwLock};

/// A layered costmap that applies a stack of layers to a master grid.
///
/// Each layer implements `CostmapLayer` and writes into the shared master grid.
/// Layers are applied in order: static → obstacle → inflation (matching Nav2).
pub struct LayeredCostmap {
    /// The master costmap grid that layers write into.
    master_grid: Arc<RwLock<Costmap>>,
    /// Ordered list of costmap layers.
    layers: Vec<Box<dyn CostmapLayer>>,
    /// Whether to reset master grid before each update cycle.
    reset_on_update: bool,
}

impl LayeredCostmap {
    /// Create a new layered costmap wrapping an existing costmap.
    pub fn new(master_grid: Arc<RwLock<Costmap>>) -> Self {
        Self {
            master_grid,
            layers: Vec::new(),
            reset_on_update: true,
        }
    }

    /// Create a new layered costmap with a freshly allocated master grid.
    pub fn new_with_size(
        width: usize,
        height: usize,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
        is_global: bool,
    ) -> Self {
        let mut costmap = Costmap::new(is_global);
        costmap.width = width;
        costmap.height = height;
        costmap.resolution = resolution;
        costmap.origin_x = origin_x;
        costmap.origin_y = origin_y;
        costmap.data = vec![crate::navigation::costmap::cost_values::NO_COST; width * height];

        Self {
            master_grid: Arc::new(RwLock::new(costmap)),
            layers: Vec::new(),
            reset_on_update: true,
        }
    }

    /// Add a layer to the costmap stack.
    pub fn add_layer(&mut self, layer: Box<dyn CostmapLayer>) {
        self.layers.push(layer);
    }

    /// Get a reference to the master grid.
    pub fn master_grid(&self) -> Arc<RwLock<Costmap>> {
        Arc::clone(&self.master_grid)
    }

    /// Update all layers for the given robot pose.
    ///
    /// This resets the master grid, then calls `update_bounds` and `update_costs`
    /// on each layer in order.
    pub fn update(&mut self, robot_x: f64, robot_y: f64, robot_yaw: f64) {
        // Compute the bounding box that needs updating
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for layer in &mut self.layers {
            layer.update_bounds(
                robot_x, robot_y, robot_yaw,
                &mut min_x, &mut min_y, &mut max_x, &mut max_y,
            );
        }

        // Reset master grid within bounds if configured
        if self.reset_on_update {
            if let Ok(mut grid) = self.master_grid.write() {
                let (gx_min, gy_min) = grid.world_to_map(min_x, min_y);
                let (gx_max, gy_max) = grid.world_to_map(max_x, max_y);

                let gx_min = gx_min.max(0) as usize;
                let gy_min = gy_min.max(0) as usize;
                let gx_max = (gx_max as usize).min(grid.width.saturating_sub(1));
                let gy_max = (gy_max as usize).min(grid.height.saturating_sub(1));

                for y in gy_min..=gy_max {
                    for x in gx_min..=gx_max {
                        let idx = y * grid.width + x;
                        grid.data[idx] = crate::navigation::costmap::cost_values::NO_COST;
                    }
                }
            }
        }

        // Apply each layer's costs to the master grid
        for layer in &self.layers {
            if let Ok(mut grid) = self.master_grid.write() {
                layer.update_costs(&mut grid, min_x, min_y, max_x, max_y);
            }
        }
    }

    /// Reset all layers.
    pub fn reset_layers(&mut self) {
        for layer in &mut self.layers {
            layer.reset();
        }
    }

    /// Configure all layers with a shared param map.
    pub fn configure_layers(&mut self, params: &ParamMap) {
        for layer in &mut self.layers {
            let name = layer.name().to_string();
            layer.configure(&name, params);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::costmap::cost_values;

    #[test]
    fn test_layered_costmap_creation() {
        let lc = LayeredCostmap::new_with_size(100, 100, 0.05, -2.5, -2.5, false);
        let binding = lc.master_grid();
        let grid = binding.read().unwrap();
        assert_eq!(grid.width, 100);
        assert_eq!(grid.height, 100);
        assert_eq!(grid.data.len(), 10000);
        // All cells should be free
        assert!(grid.data.iter().all(|&c| c == cost_values::NO_COST));
    }

    #[test]
    fn test_layered_costmap_add_layers() {
        let mut lc = LayeredCostmap::new_with_size(10, 10, 0.05, 0.0, 0.0, false);
        let sl = static_layer::StaticLayer::new();
        lc.add_layer(Box::new(sl));
        assert_eq!(lc.layers.len(), 1);
    }
}
