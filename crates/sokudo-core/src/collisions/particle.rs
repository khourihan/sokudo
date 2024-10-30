use sokudo_io::read::collider::ParsedParticle;

#[derive(Debug)]
pub struct Particle {
    /// The inverse mass of this particle.
    pub inverse_mass: f32,
}

impl Particle {
    #[inline]
    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }
}

impl From<ParsedParticle> for Particle {
    fn from(value: ParsedParticle) -> Self {
        Particle {
            inverse_mass: 1.0 / value.mass,
        }
    }
}
