// Compile directly against the deployed-baseline decoder; no duplicated math or
// Cargo dependencies. After 25 constant blocks the median filters have converged.
mod model { pub const NUM_JOINTS: usize = 15; }
#[path = "../../../duck-control/src/imu.rs"]
mod imu;
fn main() {
    let raw: [u8;12] = [100,0,56,255,44,1,0,52,0,182,0,48];
    let mut decoder = imu::SflpDecoder::default();
    let mut out = imu::ImuData::default();
    for _ in 0..25 { out = decoder.decode(&raw); }
    println!("{{\"raw_hex\":\"640038ff2c01003400b60030\",\"gyro\":{:?},\"quat\":{:?},\"gravity\":{:?},\"ready\":{}}}",out.gyro,out.quat,out.gravity,decoder.ready());
}
