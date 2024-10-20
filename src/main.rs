use std::path::PathBuf;

use clap::Parser;
use sokudo_core::run::run_simulation;

#[derive(clap::Parser)]
#[command(author, version, about)]
struct Cli {
    #[command(subcommand)]
    command: Commands
}

#[derive(clap::Subcommand)]
enum Commands {
    Run {
        /// The file to read as the initial world state.
        world: PathBuf,

        /// The file to output the simulation data to.
        #[arg(short, long, value_name = "FILENAME")]
        outfile: PathBuf,
    }
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Run {
            world,
            outfile,
        } => {
            match run_simulation(world, outfile) {
                Ok(_) => (),
                Err(err) => {
                    println!("{}", err);
                },
            }
        },
    }
}
