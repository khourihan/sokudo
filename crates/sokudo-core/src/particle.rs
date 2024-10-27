use sokudo_io::read::collider::ParsedParticle;

#[derive(Debug)]
pub struct Particle {
    /// The mass of this particle.
    pub mass: f32,
}

impl Particle {
    pub fn inverse_mass(&self) -> f32 {
        1.0 / self.mass
    }
}

impl From<ParsedParticle> for Particle {
    fn from(value: ParsedParticle) -> Self {
        Particle {
            mass: value.mass,
        }
    }
}
