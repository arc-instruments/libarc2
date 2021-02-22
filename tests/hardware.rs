#[cfg(test)]
mod hardware {
    use libarc2;
    use std::env;
    use serial_test::serial;

    #[test]
    #[serial]
    #[cfg_attr(not(feature = "hardware_tests"), ignore)]
    fn open_device() {

        println!("\nRunning open_device 0");

        match libarc2::Instrument::open(0i32) {
            Ok(_) => { println!("Device opened successfully"); }
            Err(err) => { panic!("Could not open device: {}", err); }
        }

    }

    #[test]
    #[serial]
    #[cfg_attr(not(feature = "hardware_tests"), ignore)]
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
    #[cfg_attr(not(feature = "hardware_tests"), ignore)]
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
    #[cfg_attr(not(feature = "hardware_tests"), ignore)]
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
}
