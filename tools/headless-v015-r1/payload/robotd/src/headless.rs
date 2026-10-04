//! Daemon-only policy/capability guard for the fixed-head prototype.
//! The public 0.15.0 protocol types and its shared TOML schema are unchanged.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use duck_control::io::{RobotIo, Sensors};
use duck_control::morphology;
use duck_control::safety::Safety;
use crate::{Intents, RobotState, proto};

// 0 = clear, 1 = fallen, 2 = frozen IMU, 3 = repeated bus failures.
static FAULT: AtomicU8 = AtomicU8::new(0);
static TORQUE_CUT: AtomicBool = AtomicBool::new(false);
static REARM: AtomicBool = AtomicBool::new(false);

pub fn fault_reason(code: u8) -> Option<&'static str> {
    match code {
        0 => None,
        1 => Some("headless fall latch: support and upright the robot, then explicitly rearm"),
        2 => Some("headless repeated-IMU-block latch: verify sample freshness and quantization before rearming"),
        _ => Some("headless bus fault latch: all configured physical servos must answer"),
    }
}

pub fn latched() -> bool { FAULT.load(Ordering::Acquire) != 0 }

fn cause(fallen: bool, stale: u64, errors: u64, limit: u64, stale_limit: Option<u64>) -> u8 {
    if errors >= limit { 3 }
    else if stale_limit.is_some_and(|n| stale >= n) { 2 }
    else if fallen { 1 } else { 0 }
}

fn upright_and_still(s: Option<&Sensors>) -> bool {
    s.is_some_and(|s| {
        s.imu.gravity.iter().all(|x| x.is_finite())
            && s.imu.gravity[2] < -0.94
            && s.imu.gyro.iter().all(|x| x.is_finite())
            && s.imu.gyro.iter().map(|x| x*x).sum::<f64>() < 0.09
    })
}

/// Run in the owner of the bus, before taking the command snapshot. Torque-off
/// is retried on failure, not silently treated as success. Rearming NEVER
/// enables torque or the policy; a separate explicit init/Start is necessary.
pub fn enforce_stop<T: RobotIo>(
    safety: &mut Safety<T>, state: &RobotState, intents: &Intents,
    fresh: Option<&Sensors>,
) -> bool {
    if !morphology::current().headless() { return false; }
    let error_count = state.consecutive_errors.load(Ordering::Relaxed);
    let why = cause(safety.imu_ready() && safety.fallen(), safety.imu_stale().run,
                    u64::from(error_count), u64::from(state.max_consecutive_errors.max(1)),
                    morphology::current().stop_on_identical_imu_blocks);
    if why != 0 && FAULT.compare_exchange(0, why, Ordering::AcqRel, Ordering::Acquire).is_ok() {
        tracing::error!(reason = fault_reason(why).unwrap_or("fault"), "headless motion latched off");
        TORQUE_CUT.store(false, Ordering::Release);
    }
    if REARM.swap(false, Ordering::AcqRel) {
        if why == 0 && error_count == 0 && safety.imu_ready()
            && !intents.enabled() && upright_and_still(fresh)
            && (!latched() || TORQUE_CUT.load(Ordering::Acquire)) {
            FAULT.store(0, Ordering::Release);
            TORQUE_CUT.store(false, Ordering::Release);
            tracing::warn!("headless latch cleared; torque stays OFF; init/Start is still required");
        } else {
            tracing::warn!("headless rearm refused: need fresh, upright, still sensors and acknowledged torque-off");
        }
    }
    if !latched() { return false; }
    intents.set_enabled(false);
    // Drop a pre-existing init request as well as refusing new ones at the IPC door.
    let _ = intents.take_power_request();
    if !TORQUE_CUT.load(Ordering::Acquire) {
        match safety.set_torque(false) {
            Ok(()) => TORQUE_CUT.store(true, Ordering::Release),
            Err(e) => tracing::warn!(error = %e, "headless torque-off incomplete; will retry"),
        }
    }
    true
}

/// Both JSON-RPC requests and notifications pass this gate. The controller and
/// final hardware writer independently enforce the fixed slots as defense in depth.
pub fn refusal(state: &RobotState, intents: &Intents, call: &proto::Call) -> Option<&'static str> {
    let m = morphology::current();
    if !m.headless() { return None; }
    match call {
        proto::Call::RobotHead(_) | proto::Call::RobotLook(_) =>
            Some("neck/head joints are not installed; fixed virtual pose is configured at startup"),
        proto::Call::RobotPose(_) => Some("body-pose mode is disabled for initial fixed-head validation"),
        proto::Call::RobotMouth(_) if !m.mouth_present => Some("mouth ID 34 is not installed"),
        proto::Call::RobotRebootMotors(p) if p.ids.iter().any(|id| !m.motor_ids().contains(id)) =>
            Some("reboot list contains an ID that is not a configured physical servo"),
        proto::Call::RobotDo(_) | proto::Call::RobotSetSkill(_) =>
            Some("whole-body skills are disabled on the fixed-head prototype"),
        proto::Call::RobotSetMode(_) => Some("drive-mode switching is disabled on the fixed-head prototype"),
        proto::Call::RobotChorale(p) if p.active => Some("chorale head motion is disabled on this hardware"),
        proto::Call::RobotInit => motion_refusal(state, m.motion_allowed()),
        proto::Call::RobotEnable(p) => {
            let on = if p.toggle { !intents.enabled() } else { p.on };
            if on { motion_refusal(state, m.motion_allowed()) } else { None }
        }
        _ => None,
    }
}

fn motion_refusal(state: &RobotState, allowed: bool) -> Option<&'static str> {
    if !allowed { return Some("bench-only profile: set allow_motion=true and restart after the protected checks"); }
    if let Some(reason) = fault_reason(FAULT.load(Ordering::Acquire)) { return Some(reason); }
    if state.fallen.load(Ordering::Relaxed) { return Some("upright the fixed-head robot before enabling motion"); }
    if !state.imu_ready.load(Ordering::Relaxed) { return Some("IMU is not ready"); }
    if state.consecutive_errors.load(Ordering::Relaxed) > 0 { return Some("motor bus is not currently healthy"); }
    None
}

/// Local Unix-socket extension; no new Call variant and no required rebuild of
/// robotctl, padd, btd or the shared Params consumers. Notifications do not rearm.
pub fn response(state: &RobotState, intents: &Intents, id: proto::Id, method: &str) -> proto::Response {
    if method == "robot.morphology.rearm" {
        let result = if !morphology::current().headless() {
            proto::IntentResult::refused("not a headless profile")
        } else if intents.enabled() {
            proto::IntentResult::refused("disable/relax first; rearming never enables motion")
        } else {
            REARM.store(true, Ordering::Release);
            // The control thread must decide against a fresh sensor sample.
            proto::IntentResult::accepted()
        };
        return proto::Response::ok(Some(id), &result);
    }
    let mut report = morphology::current().report();
    report["fault_latched"] = serde_json::json!(latched());
    report["fault_reason"] = serde_json::json!(fault_reason(FAULT.load(Ordering::Acquire)));
    report["torque_off_acknowledged"] = serde_json::json!(TORQUE_CUT.load(Ordering::Acquire));
    report["rearm_pending"] = serde_json::json!(REARM.load(Ordering::Acquire));
    report["policy_enabled"] = serde_json::json!(intents.enabled());
    report["imu_ready"] = serde_json::json!(state.imu_ready.load(Ordering::Relaxed));
    proto::Response::ok(Some(id), &report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stop_causes_are_explicit() {
        assert_eq!(cause(false, 24, 9, 10, Some(25)), 0);
        assert_eq!(cause(true, 0, 0, 10, None), 1);
        assert_eq!(cause(false, 25, 0, 10, Some(25)), 2);
        assert_eq!(cause(false, 0, 10, 10, None), 3);
        assert_eq!(cause(false, 1000, 0, 10, None), 0, "repeated values need not be stale");
    }
    #[test]
    fn rearm_needs_real_upright_still_sample() {
        assert!(!upright_and_still(None));
        let mut s = Sensors::default(); s.imu.gravity=[0.0,0.0,-1.0]; s.imu.gyro=[0.0;3];
        assert!(upright_and_still(Some(&s)));
        s.imu.gyro[0]=0.4; assert!(!upright_and_still(Some(&s)));
        s.imu.gyro=[0.0;3]; s.imu.gravity[2]=0.0; assert!(!upright_and_still(Some(&s)));
        s.imu.gravity[2]=f64::NAN; assert!(!upright_and_still(Some(&s)));
    }
}
