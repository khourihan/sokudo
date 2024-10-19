use sokudo_io::ParsedWorld;

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
