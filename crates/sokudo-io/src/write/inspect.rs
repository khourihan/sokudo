use std::collections::HashMap;

use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Default, Serialize, Deserialize)]
pub struct InspectElements {
    pub elements: HashMap<String, InspectFeature>,
    index: u32,
}

#[derive(Serialize, Deserialize, Clone)]
pub enum InspectFeature {
    Point(Vec3),
    Ray {
        origin: Vec3,
        direction: Vec3,
    },
    Line {
        p1: Vec3,
        p2: Vec3,
    },
}

impl InspectElements {
    pub fn reset(&mut self) {
        self.elements.clear();
        self.index = 0;
    }

    pub fn add_point(&mut self, point: Vec3) {
        self.add_named_point(self.index, point);
        self.index += 1;
    }

    pub fn add_ray(&mut self, origin: Vec3, direction: Vec3) {
        self.add_named_ray(self.index, origin, direction);
        self.index += 1;
    }

    pub fn add_line(&mut self, p1: Vec3, p2: Vec3) {
        self.add_named_line(self.index, p1, p2);
        self.index += 1;
    }

    pub fn add_named_point<S: ToString>(&mut self, name: S, point: Vec3) {
        self.elements.insert(name.to_string(), InspectFeature::Point(point));
    }

    pub fn add_named_ray<S: ToString>(&mut self, name: S, origin: Vec3, direction: Vec3) {
        self.elements.insert(name.to_string(), InspectFeature::Ray { origin, direction });
    }

    pub fn add_named_line<S: ToString>(&mut self, name: S, p1: Vec3, p2: Vec3) {
        self.elements.insert(name.to_string(), InspectFeature::Line { p1, p2 });
    }
}
