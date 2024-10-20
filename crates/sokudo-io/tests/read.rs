use sokudo_io::read::ParsedWorld;

#[test]
fn read() {
    let world = match ParsedWorld::read("tests/test-world.ron") {
        Ok(w) => w,
        Err(err) => {
            println!("{}", err);
            return;
        },
    };

    println!("world: {:?}", world);
}
