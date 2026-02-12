//! Static costmap layer — loads occupancy from a static map.
//!
//! This layer subscribes to (or is fed) a static map and writes occupancy values
//! into the master grid. It matches Nav2's `StaticLayer`.

use crate::navigation::costmap::{cost_values, Costmap};
use crate::navigation::traits::CostmapLayer;
use crate::navigation::types::ParamMap;

/// Static layer: writes a static occupancy map into the master grid.
pub struct StaticLayer {
    name: String,
    /// Stored map data (occupancy grid values 0-255).
    map_data: Vec<u8>,
    map_width: usize,
    map_height: usize,
    map_resolution: f64,
    map_origin_x: f64,
    map_origin_y: f64,
    has_map: bool,
    /// Threshold above which a cell is considered lethal (default: 65 in Nav2).
    lethal_threshold: u8,
    /// Whether to track unknown space.
    track_unknown: bool,
}

impl StaticLayer {
    pub fn new() -> Self {
        Self {
            name: "static_layer".to_string(),
            map_data: Vec::new(),
            map_width: 0,
            map_height: 0,
            map_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            has_map: false,
            lethal_threshold: 65,
            track_unknown: true,
        }
    }

    /// Update the stored static map data.
    pub fn update_map(
        &mut self,
        data: &[u8],
        width: usize,
        height: usize,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
    ) {
        self.map_data = data.to_vec();
        self.map_width = width;
        self.map_height = height;
        self.map_resolution = resolution;
        self.map_origin_x = origin_x;
        self.map_origin_y = origin_y;
        self.has_map = true;
    }

    /// Convert an occupancy value (0-100, 255=unknown) to a costmap cost.
    fn occupancy_to_cost(&self, occupancy: u8) -> u8 {
        match occupancy {
            255 => {
                if self.track_unknown {
                    cost_values::UNKNOWN_COST
                } else {
                    cost_values::NO_COST
                }
            }
            v if v >= self.lethal_threshold => cost_values::LETHAL_OBSTACLE,
            0 => cost_values::NO_COST,
            v => {
                // Scale linearly from NO_COST to just below INSCRIBED
                let scaled = (v as f64 / self.lethal_threshold as f64) * cost_values::MEDIUM_COST as f64;
                scaled.min(cost_values::MEDIUM_COST as f64) as u8
            }
        }
    }
}

impl CostmapLayer for StaticLayer {
    fn configure(&mut self, name: &str, params: &ParamMap) {
        self.name = name.to_string();

        if let Some(pv) = params.get("lethal_threshold") {
            if let Some(v) = pv.as_int() {
                self.lethal_threshold = v.clamp(0, 255) as u8;
            }
        }
        if let Some(pv) = params.get("track_unknown") {
            if let Some(v) = pv.as_bool() {
                self.track_unknown = v;
            }
        }
    }

    fn update_bounds(
        &mut self,
        _robot_x: f64,
        _robot_y: f64,
        _robot_yaw: f64,
        min_x: &mut f64,
        min_y: &mut f64,
        max_x: &mut f64,
        max_y: &mut f64,
    ) {
        if !self.has_map {
            return;
        }

        // The static layer covers the entire map area
        let map_min_x = self.map_origin_x;
        let map_min_y = self.map_origin_y;
        let map_max_x = self.map_origin_x + self.map_width as f64 * self.map_resolution;
        let map_max_y = self.map_origin_y + self.map_height as f64 * self.map_resolution;

        *min_x = min_x.min(map_min_x);
        *min_y = min_y.min(map_min_y);
        *max_x = max_x.max(map_max_x);
        *max_y = max_y.max(map_max_y);
    }

    fn update_costs(
        &self,
        master_grid: &mut Costmap,
        _min_x: f64,
        _min_y: f64,
        _max_x: f64,
        _max_y: f64,
    ) {
        if !self.has_map {
            return;
        }

        // Write static map data into the master grid.
        // We iterate the static map and project each cell into the master grid.
        for my in 0..self.map_height {
            for mx in 0..self.map_width {
                let map_idx = my * self.map_width + mx;
                let occupancy = self.map_data[map_idx];
                let cost = self.occupancy_to_cost(occupancy);

                // Convert map cell center to world coordinates
                let wx = self.map_origin_x + (mx as f64 + 0.5) * self.map_resolution;
                let wy = self.map_origin_y + (my as f64 + 0.5) * self.map_resolution;

                // Convert to master grid coordinates
                let (gx, gy) = master_grid.world_to_map(wx, wy);
                if gx >= 0
                    && gx < master_grid.width as i32
                    && gy >= 0
                    && gy < master_grid.height as i32
                {
                    let master_idx = gy as usize * master_grid.width + gx as usize;
                    // Use maximum combination: keep the higher cost
                    if cost > master_grid.data[master_idx] || cost == cost_values::LETHAL_OBSTACLE {
                        master_grid.data[master_idx] = cost;
                    }
                }
            }
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        self.map_data.clear();
        self.has_map = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_static_layer_occupancy_to_cost() {
        let layer = StaticLayer::new();
        assert_eq!(layer.occupancy_to_cost(0), cost_values::NO_COST);
        assert_eq!(layer.occupancy_to_cost(100), cost_values::LETHAL_OBSTACLE);
        assert_eq!(layer.occupancy_to_cost(255), cost_values::UNKNOWN_COST);
        assert_eq!(layer.occupancy_to_cost(65), cost_values::LETHAL_OBSTACLE);
        // Below threshold should be non-lethal
        let c = layer.occupancy_to_cost(30);
        assert!(c < cost_values::INSCRIBED_INFLATED_OBSTACLE);
        assert!(c > cost_values::NO_COST);
    }

    #[test]
    fn test_static_layer_update_map() {
        let mut layer = StaticLayer::new();
        assert!(!layer.has_map);
        layer.update_map(&[0, 100, 0, 0], 2, 2, 0.05, 0.0, 0.0);
        assert!(layer.has_map);
        assert_eq!(layer.map_width, 2);
        assert_eq!(layer.map_height, 2);
    }

    #[test]
    fn test_static_layer_writes_to_master() {
        let mut layer = StaticLayer::new();
        // A 3x3 map with an obstacle at center
        let map_data = vec![
            0, 0, 0,
            0, 100, 0,
            0, 0, 0,
        ];
        layer.update_map(&map_data, 3, 3, 1.0, 0.0, 0.0);

        // Master grid: 3x3 at same resolution
        let mut master = Costmap::new(false);
        master.width = 3;
        master.height = 3;
        master.resolution = 1.0;
        master.origin_x = 0.0;
        master.origin_y = 0.0;
        master.data = vec![cost_values::NO_COST; 9];

        layer.update_costs(&mut master, 0.0, 0.0, 3.0, 3.0);

        // Cell (1,1) should be LETHAL
        assert_eq!(master.data[1 * 3 + 1], cost_values::LETHAL_OBSTACLE);
        // Cell (0,0) should be free
        assert_eq!(master.data[0], cost_values::NO_COST);
    }
}
