//! Offline only. Takes captured/inferred raw payload records on stdin; never opens IO.
use duck_control::imu::SflpDecoder;
use duck_control::imu_quality::{Config, Mode};
use serde::Deserialize;
use std::io::BufRead;

#[derive(Deserialize)]
struct Input {
    raw: [u8; 12],
    host_read_end_ns: u64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut decoder = SflpDecoder::default();
    if std::env::args().any(|s| s == "--predict") {
        decoder.configure_quality(Config {
            mode: Mode::Predict,
            max_predict_ns: if std::env::args().any(|s| s == "--bridge-1000ms") {
                1_000_000_000
            } else {
                Config::default().max_predict_ns
            },
            ..Default::default()
        });
    }
    for line in std::io::stdin().lock().lines() {
        let row: Input = serde_json::from_str(&line?)?;
        let (_, decoded) = decoder.decode_at(&row.raw, row.host_read_end_ns);
        println!(
            "{}",
            serde_json::json!({
                "schema_version":2,"device_id":200,"register_address":124,
                "host_read_end_ns":row.host_read_end_ns,"raw":row.raw,
                "decoded":decoded,"dropped_records":0
            })
        );
    }
    Ok(())
}
