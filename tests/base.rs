use libarc2;
use std::env;
use serial_test::serial;


fn enumerate_and_open(id: i32) -> libarc2::Instrument {
    let count = libarc2::Instrument::find().unwrap();
    assert!(count > 0);

    match libarc2::Instrument::open(id) {
        Ok(d) => d,
        Err(err) => { panic!("Could not open device 0: {}", err) }
    }
}


#[test]
#[serial]
fn find_devices() {

    println!("\nRunning find_devices");

    let count = libarc2::Instrument::find();

    match count {
        Ok(count) => { println!("{} devices found", count); }
        Err(err) => { panic!("Could not enumerate devices: {}", err); }
    }

}

#[test]
#[serial]
fn open_device() {

    println!("\nRunning open_device 0");

    let _device = match libarc2::Instrument::open(0) {
        Ok(_) => { println!("Device opened succesfully") },
        Err(err) => { panic!("Could not open device 0: {}", err) }

    };

}

#[test]
#[serial]
fn load_firmware() {
    println!("\nRunning load_firmware");

    let path = match env::var("ARC2_FIRMWARE") {
        Ok(p) => p,
        Err(_) => {
            // file not found; but that's not an error
            // exit quietly
            println!("No firmware specified; exiting...");
            return;
        }
    };

    let device = enumerate_and_open(0i32);

    println!("Device opened succesfully");
    println!("Loading firmware from: {}", path);

    match device.load_firmware(&path) {
        Ok(_) => { println!("Firmware loaded succesfully") }
        Err(err) => { panic!("Could not load firmware: {}", err) }
    };
}

#[test]
#[serial]
fn reset_dacs() {
    println!("\nRunning reset_dacs");

    let path = match env::var("ARC2_FIRMWARE") {
        Ok(p) => p,
        Err(_) => {
            // file not found; but that's not an error
            // exit quietly
            println!("No firmware specified; exiting...");
            return;
        }
    };

    let device = enumerate_and_open(0i32);

    println!("Device opened succesfully");
    println!("Loading firmware from: {}", path);

    match device.load_firmware(&path) {
        Ok(_) => { println!("Firmware loaded succesfully") }
        Err(err) => { panic!("Could not load firmware: {}", err) }
    };

    match device.reset_dacs() {
        Ok(_) => { println!("RESET_DAC buffer written succesfully") }
        Err(err) => { panic!("Could not write RESET_DAC buffer: {}", err) }
    }
}
