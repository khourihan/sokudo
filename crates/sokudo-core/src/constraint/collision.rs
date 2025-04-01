use glam::{Mat3, Vec3};

use crate::collisions::{
    collider::{Collider, ColliderBody, ColliderId},
    contact::ContactData,
    rigid_body::{InertiaTensor, RigidBody},
};

use super::{Constraint, VelocityConstraint};

#[derive(Debug, Clone)]
pub struct RigidBodyCollisionConstraint {
    /// The id of the first body.
    pub a: ColliderId,
    /// The id of the second body.
    pub b: ColliderId,

    /// The contact point relative to the first body's center of mass in global space.
    pub anchor1: Vec3,
    /// The contact point relative to the second body's center of mass in global space.
    pub anchor2: Vec3,

    pub point1: Vec3,
    pub point2: Vec3,

    /// The depth of the penetration.
    pub depth: f32,
    /// The contact normal in global space, pointing from the first body to the second body.
    pub normal: Vec3,

    pub restitution: f32,
    pub stiffness: f32,
    pub friction: f32,
}

impl RigidBodyCollisionConstraint {
    /// Create a new rigid body collision constraint.
    pub fn new(
        a_id: ColliderId,
        b_id: ColliderId,
        a_body: &RigidBody,
        b_body: &RigidBody,
        contact: &ContactData,
        restitution: f32,
        friction: f32,
        stiffness: f32,
    ) -> Self {
        let anchor1 = a_body.rotation * (contact.point1 - a_body.center_of_mass);
        let anchor2 = b_body.rotation * (contact.point2 - b_body.center_of_mass);
        let normal = a_body.rotation * contact.normal1;

        Self {
            a: a_id,
            b: b_id,
            anchor1,
            anchor2,
            depth: contact.depth,
            normal,
            restitution,
            friction,
            stiffness,
            point1: a_body.rotation * contact.point1,
            point2: b_body.rotation * contact.point2,
        }
    }
}

impl Constraint for RigidBodyCollisionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn c(&self, _a: &Collider, _b: &Collider) -> f32 {
        self.depth
    }

    fn c_gradients(&self, _a: &Collider, _b: &Collider) -> (Vec3, Vec3) {
        (self.normal, -self.normal)
    }

    fn inverse_masses(&self, a: &Collider, b: &Collider) -> (f32, f32) {
        let w1 = if a.locked {
            0.0
        } else {
            a.body.positional_inverse_mass(self.anchor1, self.normal)
        };
        let w2 = if b.locked {
            0.0
        } else {
            b.body.positional_inverse_mass(self.anchor2, self.normal)
        };

        (w1, w2)
    }

    #[inline]
    fn anchors(&self, _a: &Collider, _b: &Collider) -> (Vec3, Vec3) {
        (self.anchor1, self.anchor2)
    }

    fn compliance(&self) -> f32 {
        0.0
    }
}

impl VelocityConstraint for RigidBodyCollisionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn solve(&self, a: &mut Collider, b: &mut Collider) {
        let x0 = a.center_of_mass();
        let x1 = b.center_of_mass();

        let (ColliderBody::Rigid(body0), ColliderBody::Rigid(body1)) = (&mut a.body, &mut b.body) else {
            unreachable!()
        };

        let rb0_v = body0.angular_velocity.cross(self.anchor1) + a.velocity;
        let rb1_v = body1.angular_velocity.cross(self.anchor2) + b.velocity;

        let v_rel = rb0_v - rb1_v;
        let v_rel_n = self.normal.dot(v_rel);

        let mut t = v_rel - v_rel_n * self.normal;
        let tl2 = t.length_squared();
        if tl2 > 1e-6 {
            t *= tl2.sqrt().recip();
        }

        let w1 = if a.locked {
            0.0
        } else {
            body0.positional_inverse_mass(self.anchor1, self.normal)
        };
        let w2 = if b.locked {
            0.0
        } else {
            body1.positional_inverse_mass(self.anchor2, self.normal)
        };

        let k1 = compute_k_matrix(w1, self.point1, x0, &body0.inertia_tensor);
        let k2 = compute_k_matrix(w2, self.point2, x1, &body1.inertia_tensor);
        let k = k1 + k2;

        let nkn_inv = self.normal.dot(k * self.normal).recip();
        let pmax = t.dot(k * t).recip() * v_rel.dot(t);
        let goal_v_rel_n = if v_rel_n < 0.0 {
            -self.restitution * v_rel_n
        } else {
            0.0
        };

        let delta_v_rel_n = goal_v_rel_n - v_rel_n;
        let mut correction_mag = nkn_inv * delta_v_rel_n;

        if correction_mag < 0.0 {
            correction_mag = 0.0;
        }

        if self.depth > 0.0 {
            correction_mag -= self.stiffness * nkn_inv * self.depth;
        }

        let mut p = correction_mag * self.normal;
        let pn = p.dot(self.normal);

        if self.friction * pn > pmax {
            p -= pmax * t;
        } else if self.friction * pn < -pmax {
            p += pmax * t;
        } else {
            p -= self.friction * pn * t;
        }

        if w1 != 0.0 {
            a.velocity += w1 * p;
            body0.angular_velocity += body0.global_inverse_inertia() * self.anchor1.cross(p);
        }

        if w2 != 0.0 {
            b.velocity += w2 * -p;
            body1.angular_velocity += body1.global_inverse_inertia() * self.anchor2.cross(-p);
        }
    }
}

fn compute_k_matrix(w: f32, r: Vec3, x: Vec3, i: &InertiaTensor) -> Mat3 {
    if w != 0.0 {
        let v = r - x;

        let j11 = i[(0, 0)];
        let j12 = i[(0, 1)];
        let j13 = i[(0, 2)];
        let j22 = i[(1, 1)];
        let j23 = i[(1, 2)];
        let j33 = i[(2, 2)];

        let a = v.x;
        let b = v.y;
        let c = v.z;

        let k00 = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + w;
        let k01 = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
        let k02 = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
        let k10 = k01;
        let k11 = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + w;
        let k12 = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
        let k20 = k02;
        let k21 = k12;
        let k22 = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + w;

        Mat3::from_cols(
            Vec3::new(k00, k10, k20),
            Vec3::new(k01, k11, k21),
            Vec3::new(k02, k12, k22),
        )
    } else {
        Mat3::ZERO
    }
}
