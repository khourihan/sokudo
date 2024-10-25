use glam::{Mat3, Quat, Vec3};
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::collider::Collider;

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,
    pub inspector: InspectElements,
}

impl World {
    pub fn initialize(&mut self) {
        for collider in self.colliders.iter_mut() {
            collider.set_starting_points();
            collider.compute_moments();
        }
    }

    pub fn step(&mut self) {
        self.inspector.reset();

        // Update positions according to velocity verlet
        for collider in self.colliders.iter_mut() {
            let vh = 0.5 * collider.acceleration * self.dt + collider.velocity;
            collider.transform.translate += vh * self.dt;
            collider.velocity = vh;

            let lh = 0.5 * collider.forces.torque * self.dt + collider.angular_momentum;
            let r = Mat3::from_quat(collider.transform.rotate);

            let i = collider.inertia_tensor(r);
            let ang_vel = i * lh;
            let rdot = Mat3::from_cols(
                Vec3::new(0.0, ang_vel.z, -ang_vel.y),
                Vec3::new(-ang_vel.z, 0.0, ang_vel.x),
                Vec3::new(ang_vel.y, -ang_vel.x, 0.0),
            ) * r;

            collider.transform.rotate = Quat::from_mat3(&(r + rdot * self.dt)).normalize();
            collider.angular_momentum = lh;
            collider.angular_velocity = ang_vel;
        }

        // Accumulate forces
        self.apply_collisions();
        self.apply_gravity();

        // Update velocity according to velocity verlet
        for collider in self.colliders.iter_mut() {
            // F = ma
            collider.acceleration = if collider.locked { Vec3::ZERO } else { collider.forces.linear / collider.mass };
            collider.velocity = 0.5 * collider.acceleration * self.dt + collider.velocity;

            let angular_acceleration = if collider.locked { Vec3::ZERO } else { collider.forces.torque };
            collider.angular_momentum = 0.5 * angular_acceleration * self.dt + collider.angular_momentum;

            // Reset forces
            collider.forces.linear = Vec3::ZERO;
            collider.forces.torque = Vec3::ZERO;
        }
    }

    fn apply_collisions(&mut self) {
        for i in 0..self.colliders.len() {
            for j in 0..self.colliders.len() {
                if i >= j {
                    continue;
                }

                unsafe {
                    let collider1 = &mut *(self.colliders.get_unchecked_mut(i) as *mut Collider);
                    let collider2 = &mut *(self.colliders.get_unchecked_mut(j) as *mut Collider);

                    collider1.collide(collider2, &mut self.inspector);
                }
            }
        }
    }

    fn apply_gravity(&mut self) {
        for collider in self.colliders.iter_mut() {
            collider.forces.linear += self.gravity * collider.mass;
        }
    }

    pub fn state(&self) -> WriteWorldState {
        WriteWorldState {
            colliders: self.colliders.iter().map(WriteCollider::from).collect(),
            inspector: self.inspector.clone(),
        }
    }
}

impl From<ParsedWorld> for World {
    fn from(value: ParsedWorld) -> Self {
        World {
            steps: value.steps,
            dt: value.dt,
            gravity: value.gravity,
            colliders: value.colliders.into_iter().map(Collider::from).collect(),
            inspector: InspectElements::default(),
        }
    }
}
