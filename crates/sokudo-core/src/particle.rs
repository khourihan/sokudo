use sokudo_io::read::collider::ParsedParticle;

#[derive(Debug)]
pub struct Particle {
    /// The mass of this particle.
    pub mass: f32,
}

impl From<ParsedParticle> for Particle {
    fn from(value: ParsedParticle) -> Self {
        Particle {
            mass: value.mass,
        }
    }
}
