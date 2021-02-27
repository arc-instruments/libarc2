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
    }
}

