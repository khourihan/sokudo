use std::{fs, path};

use collider::WriteCollider;
use inspect::InspectElements;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod collider;
pub mod inspect;
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

#[derive(Error, Debug)]
pub enum ReadStateError {
    /// An IO error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// A RON error.
    #[error(transparent)]
    Ron(#[from] ron::de::SpannedError),
}

#[derive(Serialize, Deserialize)]
pub struct WriteWorldState {
    pub colliders: Vec<WriteCollider>,
    pub inspector: InspectElements,
}

#[derive(Serialize, Default)]
#[serde(rename = "History")]
pub struct WriteWorldStateHistory {
    states: Vec<WriteWorldState>,
}

impl WriteWorldStateHistory {
    /// Push a state to this [`WriteWorldStateHistory`].
    pub fn push(&mut self, state: WriteWorldState) {
        self.states.push(state)
    }

    /// Write this [`WriteWorldStateHistory`] to file `path`.
    // TODO: Make async or Box::pin(async move ||)
    pub fn write<P>(&self, path: P) -> Result<(), WriteStateError>
    where
        P: AsRef<path::Path>,
    {
        let file = fs::File::create(path)?;
        ron::ser::to_writer(file, self)?;

        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(rename = "History")]
pub struct ReadWorldStateHistory {
    states: Vec<WriteWorldState>,
}

impl ReadWorldStateHistory {
    /// Pops a state from this [`ReadWorldStateHistory`], returning it.
    pub fn pop(&mut self) -> WriteWorldState {
        self.states.remove(0)
    }

    /// Gets a state from this [`ReadWorldStateHistory`], given its index.
    pub fn get(&self, step: usize) -> &WriteWorldState {
        self.states.get(step).as_ref().unwrap()
    }

    /// The number of states in this [`ReadWorldStateHistory`].
    pub fn len(&self) -> usize {
        self.states.len()
    }

    /// Returns whether or not this [`ReadWorldStateHistory`] is empty.
    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }

    /// Read the given `path` into a [`ReadWorldStateHistory`].
    pub fn read<P>(path: P) -> Result<ReadWorldStateHistory, ReadStateError>
    where
        P: AsRef<path::Path>,
    {
        let file = fs::File::open(path)?;
        let de: ReadWorldStateHistory = ron::de::from_reader(file)?;

        Ok(de)
    }
}
