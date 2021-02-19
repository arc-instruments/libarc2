use libarc2;
use std::env;
use serial_test::serial;
use assert_matches::assert_matches;


#[test]
#[serial]
fn open_device() {

    println!("\nRunning open_device 0");

    match libarc2::Instrument::open(0i32) {
        Ok(_) => { println!("Device opened successfully"); }
        Err(err) => { panic!("Could not open device: {}", err); }
    }

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

    let device = libarc2::Instrument::open(0i32).unwrap();

    println!("Device opened succesfully");
    println!("Loading firmware from: {}", path);

    match device.load_firmware(&path) {
        Ok(_) => { println!("Firmware loaded succesfully") }
        Err(err) => { panic!("Could not load firmware: {}", err) }
    };
}

#[test]
#[serial]
fn open_with_fw() {

    println!("\nRunning open_with_fw");

    let path = match env::var("ARC2_FIRMWARE") {
        Ok(p) => p,
        Err(_) => {
            // file not found; but that's not an error
            // exit quietly
            println!("No firmware specified; exiting...");
            return;
        }
    };

    let _device = libarc2::Instrument::open_with_fw(0i32, &path).unwrap();

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

    let device = libarc2::Instrument::open(0i32).unwrap();

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

#[test]
fn channel_conf() {
    let mut v = libarc2::ChannelConfRegister::new(64);
    v.set_all(libarc2::ChannelConf::VoltArb);

    for channel in &v {
        assert_matches!(channel, libarc2::ChannelConf::VoltArb);
    }

    let slice = v.as_repr();

    assert_eq!(slice[0], 0x92492492);
    assert_eq!(slice[1], 0x49249249);
    assert_eq!(slice[2], 0x24924924);
    assert_eq!(slice[3], 0x92492492);
    assert_eq!(slice[4], 0x49249249);
    assert_eq!(slice[5], 0x24924924);
}
