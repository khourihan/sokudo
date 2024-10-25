use std::collections::HashMap;

use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Default, Serialize, Deserialize)]
pub struct InspectElements {
    pub elements: HashMap<String, InspectFeature>,
}

#[derive(Serialize, Deserialize, Clone)]
pub enum InspectFeature {
    Point(Vec3),
    Ray {
        origin: Vec3,
        direction: Vec3,
    },
}

impl InspectElements {
    pub fn reset(&mut self) {
        self.elements.clear()
    }

    pub fn add_point<S: ToString>(&mut self, name: S, point: Vec3) {
        self.elements.insert(name.to_string(), InspectFeature::Point(point));
    }

    pub fn add_ray<S: ToString>(&mut self, name: S, origin: Vec3, direction: Vec3) {
        self.elements.insert(name.to_string(), InspectFeature::Ray { origin, direction });
    }
}
