//! Offline golden vectors for the Python physics harness; NEVER opens a bus.
//! JSONL on stdin -> JSONL on stdout. Uses the real Rust ABI builder, scatter and
//! morphology projection so an independently written simulator cannot silently
//! move right-leg slots or subtract HOME twice while producing plausible motion.
use duck_control::{
    imu::ImuData,
    io::Sensors,
    model::{DEFAULT_POSITION, NUM_JOINTS},
    morphology,
    obs::{ACTION_LEN, BodyPose, Command, Observation},
};
use serde::Deserialize;
use std::{
    error::Error,
    io::{self, BufRead},
};

#[derive(Deserialize)]
struct Frame {
    positions: [f64; NUM_JOINTS],
    velocities: [f64; NUM_JOINTS],
    gyro: [f64; 3],
    gravity: [f64; 3],
    previous_action: [f32; ACTION_LEN],
    action: [f32; ACTION_LEN],
    twist: [f64; 3],
    #[serde(default)]
    body: [f64; 3],
    #[serde(default)]
    standing: bool,
    // Offline sit/rise experiment only: vx encodes posture (1/0), not velocity.
    // The production headless daemon continues to refuse these whole-body skills.
    #[serde(default)]
    posture_flag: Option<bool>,
    #[serde(default)]
    previous_target: Option<[f64; NUM_JOINTS]>,
}
fn main() -> Result<(), Box<dyn Error>> {
    morphology::initialize_from_env().map_err(io::Error::other)?;
    let m = morphology::current();
    for line in io::stdin().lock().lines() {
        let f: Frame = serde_json::from_str(&line?)?;
        let mut s = Sensors {
            positions: f.positions,
            velocities: f.velocities,
            imu: ImuData {
                gyro: f.gyro,
                gravity: f.gravity,
                ..Default::default()
            },
            ..Default::default()
        };
        m.project_sensors(&mut s);
        let mut c = Command {
            twist: f.twist,
            body: BodyPose {
                z: f.body[0],
                roll: f.body[1],
                pitch: f.body[2],
            },
            ..Default::default()
        };
        m.project_command(&mut c);
        if let Some(sit) = f.posture_flag {
            c.twist = [if sit { 1.0 } else { 0.0 }, 0.0, 0.0];
        }
        let o = Observation::build(
            &s.imu,
            &s.positions,
            &s.velocities,
            &DEFAULT_POSITION,
            &f.previous_action,
            &c,
        );
        let offsets = Observation::scatter_action(&f.action);
        let scale = if f.standing { 1.0 } else { 0.9 };
        let mut targets = std::array::from_fn(|j| DEFAULT_POSITION[j] + scale * offsets[j]);
        if let Some(previous) = f.previous_target {
            for j in 0..NUM_JOINTS {
                let alpha = if (5..9).contains(&j) { 0.5 } else { 0.7 };
                targets[j] = previous[j] + alpha * (targets[j] - previous[j]);
            }
        }
        m.project_positions(&mut targets);
        let (packed, n) = m.pack_positions(&targets);
        println!(
            "{}",
            serde_json::json!({"observation": o.as_slice(), "scatter": offsets,
            "targets": targets, "active_ids": m.motor_ids(), "packed": &packed[..n]})
        );
    }
    Ok(())
}
