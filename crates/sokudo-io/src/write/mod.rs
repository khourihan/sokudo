use std::{fs, path};

use collider::WriteCollider;
use serde::Serialize;
use thiserror::Error;

pub mod collider;
pub mod transform;

#[derive(Error, Debug)]
pub enum WriteStateError {
    /// An IO error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// A RON error.
    #[error(transparent)]
    Ron(#[from] ron::Error),
}

#[derive(Serialize)]
pub struct WriteWorldState {
    pub colliders: Vec<WriteCollider>,
}

#[derive(Serialize, Default)]
pub struct WorldStateHistory {
    states: Vec<WriteWorldState>,
}

impl WorldStateHistory {
    /// Push a state to this [`WorldStateHistory`].
    pub fn push(&mut self, state: WriteWorldState) {
        self.states.push(state)
    }

    /// Write this [`WorldStateHistory`] to file `path`.
    // TODO: Make async or Box::pin(async move ||)
    pub fn write<P>(&self, path: P) -> Result<(), WriteStateError>
    where
        P: AsRef<path::Path>
    {
        let file = fs::File::create(path)?;
        ron::ser::to_writer(file, self)?;

        Ok(())
    }
}
