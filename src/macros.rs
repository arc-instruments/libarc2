macro_rules! vidx {
    ($val:expr, $offset:expr, $slope:expr) => {
        match ((($val + $offset)/($slope)) as f64).round() {
            c if c < 0.0 => 0u16,
            c if c > 65535.0 => 0xFFFFu16,
            c => c as u16
        }
    };

    ($val:expr) => {
        vidx!($val, 10.0, 3.05179e-4)
    };

    ($val:expr, $range:expr) => {
        if $range == OutputRange::EXT {
            vidx!($val, 20.0, 6.10358e-4)
        } else {
            vidx!($val, 10.0, 3.05179e-4)
        }
    }
}


#[cfg(feature="debug_packets")]
macro_rules! instrdbg {
    ($val: expr) => {
        eprintln!("INSTR: [{:>8}] {:08x?}", $val.name(), $val.view());
        eprintln!("INSTR: [{:>8}] {:02x?}", $val.name(), $val.to_bytevec());
    }
}

#[cfg(feature="debug_packets")]
macro_rules! pktdbg {
    ($val: expr) => {
        eprintln!("BUFFR: {:02x?}", $val);
    }
}

#[cfg(not(feature="debug_packets"))]
macro_rules! instrdbg {
    ($val: expr) => { }
}

#[cfg(not(feature="debug_packets"))]
macro_rules! pktdbg {
    ($val: expr) => { }
}
