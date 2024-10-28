#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Coefficient {
    pub value: f32,
    pub rule: CoefficientCombine,
}

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CoefficientCombine {
    #[default]
    Average = 1,
    Min = 2,
    Multiply = 3,
    Max = 4,
}

impl Coefficient {
    pub fn new(value: f32, rule: CoefficientCombine) -> Coefficient {
        Coefficient { value, rule }
    }

    pub fn combine(self, other: Coefficient) -> Coefficient {
        let rule = self.rule.max(other.rule);

        Coefficient {
            value: match rule {
                CoefficientCombine::Average => (self.value + other.value) * 0.5,
                CoefficientCombine::Min => self.value.min(other.value),
                CoefficientCombine::Multiply => self.value * other.value,
                CoefficientCombine::Max => self.value.max(other.value),
            },
            rule
        }
    }
}

impl std::ops::Deref for Coefficient {
    type Target = f32;

    fn deref(&self) -> &Self::Target {
        &self.value
    }
}
