use std::path;

use sokudo_io::{
    read::{ParseError, ParsedWorld},
    write::{WriteStateError, WriteWorldStateHistory},
};
use thiserror::Error;

use crate::world::World;

#[derive(Error, Debug)]
pub enum RunSimulationError {
    /// A parse error.
    #[error(transparent)]
    Parse(#[from] ParseError),
    /// A state writing error.
    #[error(transparent)]
    WriteState(#[from] WriteStateError),
}

pub fn run_simulation<P>(world_path: P, state_path: P) -> Result<(), RunSimulationError>
where
    P: AsRef<path::Path>,
{
    let mut world: World = ParsedWorld::read(world_path)?.into();
    let mut history = WriteWorldStateHistory::default();

    world.initialize();
    history.push(world.state());

    for _ in 0..world.steps {
        world.step();
        history.push(world.state());
    }

    history.write(state_path)?;

    Ok(())
}
