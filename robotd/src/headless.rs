//! Daemon-only policy/capability guard for the fixed-head prototype.
//! The public 0.15.0 protocol types and its shared TOML schema are unchanged.

use crate::{Intents, RobotState, proto};
use duck_control::io::{RobotIo, Sensors};
use duck_control::morphology;
use duck_control::safety::Safety;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};

// 0 = clear, 1 = fallen, 2 = repeated IMU, 3 = bus, 4 = invalid sample, 5 = policy.
// This state is process-local. Restart only with support and motor power removed.
static FAULT: AtomicU8 = AtomicU8::new(0);
static TORQUE_CUT: AtomicBool = AtomicBool::new(false);
static REARM: AtomicBool = AtomicBool::new(false);
// 0 = not requested, 1 = queued, 2 = cleared, 3 = refused by control thread.
static REARM_RESULT: AtomicU8 = AtomicU8::new(0);

pub fn fault_reason(code: u8) -> Option<&'static str> {
    match code {
        0 => None,
        1 => Some("headless fall latch: support and upright the robot, then explicitly rearm"),
        2 => Some(
            "headless repeated-IMU-block latch: verify sample freshness and quantization before rearming",
        ),
        3 => Some("headless bus fault latch: all configured physical servos must answer"),
        4 => Some("headless invalid sensor sample: verify real joints and IMU"),
        5 => Some("headless inference fault: verify model/runtime before rearming"),
        _ => Some("headless unknown fault"),
    }
}

pub fn latched() -> bool {
    FAULT.load(Ordering::Acquire) != 0
}

pub fn current_fault_reason() -> Option<&'static str> {
    fault_reason(FAULT.load(Ordering::Acquire))
}

fn record_fault(why: u8) {
    // Preserve the first cause: a later missing read must not hide an ONNX fault.
    if why != 0
        && FAULT
            .compare_exchange(0, why, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    {
        TORQUE_CUT.store(false, Ordering::Release);
        tracing::error!(
            reason = fault_reason(why).unwrap_or("unknown"),
            "headless motion latched off"
        );
    }
}

/// A finite but enormous negative gravity z is not an upright sample. Do not
/// substitute zeros for real leg failures; only the absent slots are projected.
pub fn valid_sensors(s: &Sensors) -> bool {
    let norm2 = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>();
    s.positions
        .iter()
        .chain(&s.velocities)
        .chain(&s.currents_ma)
        .chain(&s.imu.gyro)
        .chain(&s.imu.gravity)
        .chain(&s.imu.quat)
        .all(|x| x.is_finite())
        && (0.64..=1.44).contains(&norm2(&s.imu.gravity))
        && (0.64..=1.44).contains(&norm2(&s.imu.quat))
}

fn cut_torque<T: RobotIo>(safety: &mut Safety<T>, intents: &Intents) {
    intents.set_enabled(false);
    // An init queued before the fault is not permission to enable after it.
    let _ = intents.take_power_request();
    if !TORQUE_CUT.load(Ordering::Acquire) {
        match safety.set_torque(false) {
            Ok(()) => TORQUE_CUT.store(true, Ordering::Release),
            Err(e) => tracing::warn!(error = %e, "headless torque-off incomplete; will retry"),
        }
    }
}

/// Holding the previous target is not a validated balance controller after an
/// ONNX error. Stop in this SAME tick, before writing another policy target.
pub fn inference_failed<T: RobotIo>(safety: &mut Safety<T>, intents: &Intents) {
    if morphology::current().headless() {
        record_fault(5);
        cut_torque(safety, intents);
    }
}

fn cause(fallen: bool, stale: u64, errors: u64, limit: u64, stale_limit: Option<u64>) -> u8 {
    if errors >= limit {
        3
    } else if stale_limit.is_some_and(|n| stale >= n) {
        2
    } else if fallen {
        1
    } else {
        0
    }
}

fn upright_and_still(s: Option<&Sensors>) -> bool {
    s.is_some_and(|s| {
        valid_sensors(s)
            && s.imu.gravity[2] < -0.94
            && s.imu.gyro.iter().all(|x| x.is_finite())
            && s.imu.gyro.iter().map(|x| x * x).sum::<f64>() < 0.09
    })
}

/// Run in the owner of the bus, before taking the command snapshot. Torque-off
/// is retried on failure, not silently treated as success. Rearming NEVER
/// enables torque or the policy; a separate explicit init/Start is necessary.
pub fn enforce_stop<T: RobotIo>(
    safety: &mut Safety<T>,
    state: &RobotState,
    intents: &Intents,
    fresh: Option<&Sensors>,
) -> bool {
    if !morphology::current().headless() {
        return false;
    }
    let error_count = state.consecutive_errors.load(Ordering::Relaxed);
    let mut why = cause(
        safety.imu_ready() && safety.fallen(),
        safety.imu_stale().run,
        u64::from(error_count),
        u64::from(state.max_consecutive_errors.max(1)),
        morphology::current().stop_on_identical_imu_blocks,
    );
    // Do not call unconverged orientation invalid; it already prevents enabling.
    if safety.imu_ready() && fresh.is_some_and(|s| !valid_sensors(s)) {
        why = 4;
    }
    record_fault(why);
    if REARM.swap(false, Ordering::AcqRel) {
        if latched()
            && why == 0
            && error_count == 0
            && safety.imu_ready()
            && !intents.enabled()
            && upright_and_still(fresh)
            && TORQUE_CUT.load(Ordering::Acquire)
        {
            FAULT.store(0, Ordering::Release);
            TORQUE_CUT.store(false, Ordering::Release);
            REARM_RESULT.store(2, Ordering::Release);
            tracing::warn!(
                "headless latch cleared; torque stays OFF; init/Start is still required"
            );
        } else {
            REARM_RESULT.store(3, Ordering::Release);
            tracing::warn!(
                "headless rearm refused: need fresh, upright, still sensors and acknowledged torque-off"
            );
        }
    }
    if !latched() {
        return false;
    }
    cut_torque(safety, intents);
    true
}

/// Both JSON-RPC requests and notifications pass this gate. The controller and
/// final hardware writer independently enforce the fixed slots as defense in depth.
pub fn refusal(state: &RobotState, intents: &Intents, call: &proto::Call) -> Option<&'static str> {
    let m = morphology::current();
    if !m.headless() {
        return None;
    }
    match call {
        proto::Call::RobotHead(_) | proto::Call::RobotLook(_) => {
            Some("neck/head joints are not installed; fixed virtual pose is configured at startup")
        }
        proto::Call::RobotPose(_) => {
            Some("body-pose mode is disabled for initial fixed-head validation")
        }
        proto::Call::RobotMouth(_) if !m.mouth_present => Some("mouth ID 34 is not installed"),
        proto::Call::RobotRebootMotors(p) if p.ids.iter().any(|id| !m.motor_ids().contains(id)) => {
            Some("reboot list contains an ID that is not a configured physical servo")
        }
        proto::Call::RobotDo(_) | proto::Call::RobotSetSkill(_) => {
            Some("whole-body skills are disabled on the fixed-head prototype")
        }
        proto::Call::RobotSetMode(_) => {
            Some("drive-mode switching is disabled on the fixed-head prototype")
        }
        proto::Call::RobotChorale(p) if p.active => {
            Some("chorale head motion is disabled on this hardware")
        }
        proto::Call::RobotInit => motion_refusal(state, m.motion_allowed()),
        proto::Call::RobotEnable(p) => {
            let on = if p.toggle { !intents.enabled() } else { p.on };
            if on {
                motion_refusal(state, m.motion_allowed())
            } else {
                None
            }
        }
        _ => None,
    }
}

fn motion_refusal(state: &RobotState, allowed: bool) -> Option<&'static str> {
    if !allowed {
        return Some(
            "bench-only profile: set allow_motion=true and restart after the protected checks",
        );
    }
    if let Some(reason) = fault_reason(FAULT.load(Ordering::Acquire)) {
        return Some(reason);
    }
    // A requested policy that failed to load differs from intentional --no-policy
    // bench operation. Refuse both HOME/init and enable on a broken bundle.
    if state.policy_error.load_full().is_some() {
        return Some("policy failed to load; inspect robot.health before enabling");
    }
    if state.fallen.load(Ordering::Relaxed) {
        return Some("upright the fixed-head robot before enabling motion");
    }
    if !state.imu_ready.load(Ordering::Relaxed) {
        return Some("IMU is not ready");
    }
    if state.consecutive_errors.load(Ordering::Relaxed) > 0 {
        return Some("motor bus is not currently healthy");
    }
    None
}

/// Local Unix-socket extension; no new Call variant and no required rebuild of
/// robotctl, padd, btd or the shared Params consumers. Notifications do not rearm.
pub fn response(
    state: &RobotState,
    intents: &Intents,
    id: proto::Id,
    method: &str,
) -> proto::Response {
    if method == "robot.morphology.rearm" {
        let result = if !morphology::current().headless() {
            proto::IntentResult::refused("not a headless profile")
        } else if !latched() {
            proto::IntentResult::refused("no latched fault to clear")
        } else if intents.enabled() {
            proto::IntentResult::refused("disable/relax first; rearming never enables motion")
        } else {
            REARM_RESULT.store(1, Ordering::Release);
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
    report["rearm_result"] = serde_json::json!(match REARM_RESULT.load(Ordering::Acquire) {
        1 => "queued",
        2 => "cleared",
        3 => "refused",
        _ => "not_requested",
    });
    report["latch_persistent_across_restart"] = serde_json::json!(false);
    report["policy_enabled"] = serde_json::json!(intents.enabled());
    report["imu_ready"] = serde_json::json!(state.imu_ready.load(Ordering::Relaxed));
    proto::Response::ok(Some(id), &report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_control::io::{FakeIo, IoError, JointTargets, SlowSensors};
    use duck_control::safety::SafetyConfig;
    use std::sync::{Arc, Mutex};

    // Only this test touches the global latch; the other tests exercise pure checks.
    // The fake records every attempt, including failures: an attempted OFF is not OFF.
    struct RetryIo {
        fake: FakeIo,
        attempts: Arc<Mutex<Vec<bool>>>,
        fail_off: bool,
    }
    impl RobotIo for RetryIo {
        fn read(&mut self) -> Result<Sensors, IoError> {
            self.fake.read()
        }
        fn write(&mut self, t: &JointTargets) -> Result<(), IoError> {
            self.fake.write(t)
        }
        fn set_gain(&mut self, kp: u16) -> Result<(), IoError> {
            self.fake.set_gain(kp)
        }
        fn set_torque(&mut self, on: bool) -> Result<(), IoError> {
            self.attempts.lock().unwrap().push(on);
            if !on && self.fail_off {
                self.fail_off = false;
                return Err(IoError::Simulated);
            }
            self.fake.set_torque(on)
        }
        fn reboot(&mut self, id: u8) -> Result<(), IoError> {
            self.fake.reboot(id)
        }
        fn slow_sensors(&mut self) -> Result<SlowSensors, IoError> {
            self.fake.slow_sensors()
        }
        fn imu_ready(&self) -> bool {
            self.fake.imu_ready()
        }
    }
    #[test]
    fn torque_off_failure_retries_without_enabling_and_preserves_first_fault() {
        let attempts = Arc::new(Mutex::new(Vec::new()));
        let mut safety = Safety::new(
            RetryIo {
                fake: FakeIo::new(),
                attempts: attempts.clone(),
                fail_off: true,
            },
            SafetyConfig::default(),
        );
        let intents = Intents::new();
        intents.set_enabled(true);
        record_fault(5);
        record_fault(3);
        assert_eq!(
            FAULT.load(Ordering::Acquire),
            5,
            "bus failure must not hide first ONNX failure"
        );
        cut_torque(&mut safety, &intents);
        assert!(!TORQUE_CUT.load(Ordering::Acquire));
        assert!(!intents.enabled());
        cut_torque(&mut safety, &intents);
        assert!(TORQUE_CUT.load(Ordering::Acquire));
        cut_torque(&mut safety, &intents);
        assert_eq!(*attempts.lock().unwrap(), vec![false, false]);
        // Success stops retries; it does not clear the latch or enable the policy.
        assert!(latched());
        assert!(!intents.enabled());
        FAULT.store(0, Ordering::Release);
        TORQUE_CUT.store(false, Ordering::Release);
    }
    #[test]
    fn stop_causes_are_explicit() {
        assert_eq!(cause(false, 24, 9, 10, Some(25)), 0);
        assert_eq!(cause(true, 0, 0, 10, None), 1);
        assert_eq!(cause(false, 25, 0, 10, Some(25)), 2);
        assert_eq!(cause(false, 0, 10, 10, None), 3);
        assert_eq!(
            cause(false, 1000, 0, 10, None),
            0,
            "repeated values need not be stale"
        );
    }
    #[test]
    fn rearm_needs_real_upright_still_sample() {
        assert!(!upright_and_still(None));
        let mut s = Sensors::default();
        s.imu.gravity = [0.0, 0.0, -1.0];
        s.imu.gyro = [0.0; 3];
        assert!(upright_and_still(Some(&s)));
        s.imu.gyro[0] = 0.4;
        assert!(!upright_and_still(Some(&s)));
        s.imu.gyro = [0.0; 3];
        s.imu.gravity[2] = 0.0;
        assert!(!upright_and_still(Some(&s)));
        s.imu.gravity[2] = f64::NAN;
        assert!(!upright_and_still(Some(&s)));
        s.imu.gravity[2] = -100.0;
        assert!(!upright_and_still(Some(&s)));
        s.imu.gravity[2] = -1.0;
        s.positions[12] = f64::NAN;
        assert!(!valid_sensors(&s));
    }
}
