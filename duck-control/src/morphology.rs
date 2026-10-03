//! Fixed physical topology without changing the 15-slot wire / 61 -> 14 policy ABI.
//!
//! Loaded once by robotd, before opening the bus. No hot reload, no discovery-based
//! masking: a configured leg that stops replying remains a real communication fault.

use std::path::Path;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};

use crate::io::Sensors;
use crate::model::{DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use crate::obs::{BodyPose, Command};

pub const BUILD_ID: &str = "headless-v015-r2";
pub const CONFIG_ENV: &str = "MICRODUCK_MORPHOLOGY";
pub const HEAD_SLOTS: [usize; 4] = [5, 6, 7, 8];
pub const LEG_POLICY_SLOTS: [usize; 10] = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Profile {
    Full15,
    Headless,
}

fn yes() -> bool {
    true
}
fn home_head() -> [f64; 4] {
    HEAD_SLOTS.map(|j| DEFAULT_POSITION[j])
}
fn zero() -> f64 {
    0.0
}
fn max_twist() -> [f64; 3] {
    [0.10, 0.04, 0.40]
}

/// Only the four neck/head joints may be omitted as a group. The mouth is
/// independent. There is deliberately no arbitrary `ignore_ids` option.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Morphology {
    pub schema_version: u32,
    pub profile: Profile,
    #[serde(default = "yes")]
    pub mouth_present: bool,
    /// Absolute joint angles in the existing runtime coordinate system, radians.
    /// NOT offsets and NOT Dynamixel raw counts. HOME is subtracted by obs.rs.
    #[serde(default = "home_head")]
    pub locked_head_rad: [f64; 4],
    #[serde(default = "zero")]
    pub locked_mouth_rad: f64,
    /// Bench-first opt-in. Does not certify that a policy can balance this body.
    #[serde(default)]
    pub allow_motion: bool,
    #[serde(default = "max_twist")]
    pub max_twist: [f64; 3],
    /// Identical payloads alone do not prove staleness (quantization at rest).
    /// Keep the upstream diagnostic by default. Opt into a stop only AFTER
    /// validating this threshold against the actual IMU firmware on the bench.
    #[serde(default)]
    pub stop_on_identical_imu_blocks: Option<u64>,
}

impl Default for Morphology {
    fn default() -> Self {
        Self {
            schema_version: 1,
            profile: Profile::Full15,
            mouth_present: true,
            locked_head_rad: home_head(),
            locked_mouth_rad: 0.0,
            allow_motion: true,
            max_twist: max_twist(),
            stop_on_identical_imu_blocks: None,
        }
    }
}

static ACTIVE: OnceLock<Morphology> = OnceLock::new();

pub fn current() -> &'static Morphology {
    ACTIVE.get_or_init(Morphology::default)
}

/// No environment variable means the unmodified full-robot behavior.
/// A specified missing/invalid file is fatal, never a fallback to Full15.
pub fn initialize_from_env() -> Result<(), String> {
    let cfg = match std::env::var_os(CONFIG_ENV) {
        Some(path) if !path.is_empty() => Morphology::load(Path::new(&path))?,
        Some(_) => return Err(format!("{CONFIG_ENV} must not be empty")),
        None => Morphology::default(),
    };
    cfg.validate()?;
    ACTIVE
        .set(cfg)
        .map_err(|_| "morphology initialized more than once".to_owned())
}

impl Morphology {
    pub fn load(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        if bytes.len() > 65_536 {
            return Err("morphology file exceeds 64 KiB".to_owned());
        }
        let cfg: Self =
            serde_json::from_slice(&bytes).map_err(|e| format!("{}: {e}", path.display()))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 {
            return Err("schema_version must be 1".to_owned());
        }
        if self.profile == Profile::Full15 && !self.allow_motion {
            return Err(
                "explicit full15 requires allow_motion=true; refusing a misleading disabled flag"
                    .to_owned(),
            );
        }
        if self.profile == Profile::Full15 && !self.mouth_present {
            return Err(
                "full15 requires mouth_present=true; use headless for this patch".to_owned(),
            );
        }
        // Conservative validation envelope for this patch, not a mechanical certification.
        let lo = [-1.57, -1.57, -1.57, -0.35];
        let hi = [1.57, 1.57, 1.57, 0.35];
        for (i, &q) in self.locked_head_rad.iter().enumerate() {
            if !q.is_finite() || q < lo[i] || q > hi[i] {
                return Err(format!(
                    "locked_head_rad[{i}] is outside the permitted fixed-pose envelope"
                ));
            }
        }
        if !self.locked_mouth_rad.is_finite() || !(-0.08..=0.52).contains(&self.locked_mouth_rad) {
            return Err("locked_mouth_rad must be finite in -0.08..=0.52".to_owned());
        }
        if self.stop_on_identical_imu_blocks.is_some_and(|n| n < 25) {
            return Err("stop_on_identical_imu_blocks must be null or at least 25".to_owned());
        }
        if self.max_twist.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err("max_twist must contain three positive finite limits".to_owned());
        }
        Ok(())
    }

    pub fn headless(&self) -> bool {
        self.profile == Profile::Headless
    }
    pub fn motion_allowed(&self) -> bool {
        !self.headless() || self.allow_motion
    }

    pub fn is_present(&self, joint: usize) -> bool {
        joint < NUM_JOINTS
            && (!self.headless()
                || (!HEAD_SLOTS.contains(&joint) && (joint != MOUTH_INDEX || self.mouth_present)))
    }

    pub fn active_slots(&self) -> impl Iterator<Item = usize> + '_ {
        (0..NUM_JOINTS).filter(|&j| self.is_present(j))
    }

    pub fn motor_ids(&self) -> Vec<u8> {
        self.active_slots().map(|j| JOINT_IDS[j]).collect()
    }

    pub fn is_absent_id(&self, id: u8) -> bool {
        JOINT_IDS
            .iter()
            .position(|&x| x == id)
            .is_some_and(|j| !self.is_present(j))
    }

    /// Fixed slots are configuration-derived, not sensor readings.
    pub fn project_positions(&self, positions: &mut [f64; NUM_JOINTS]) {
        if !self.headless() {
            return;
        }
        for (j, q) in HEAD_SLOTS.into_iter().zip(self.locked_head_rad) {
            positions[j] = q;
        }
        if !self.mouth_present {
            positions[MOUTH_INDEX] = self.locked_mouth_rad;
        }
    }

    pub fn seed_positions(&self) -> [f64; NUM_JOINTS] {
        let mut out = DEFAULT_POSITION;
        self.project_positions(&mut out);
        out
    }

    pub fn project_sensors(&self, sensors: &mut Sensors) {
        self.project_positions(&mut sensors.positions);
        for j in 0..NUM_JOINTS {
            if !self.is_present(j) {
                sensors.velocities[j] = 0.0;
                sensors.currents_ma[j] = 0.0;
            }
        }
    }

    /// Called AFTER skill arbitration, BEFORE Observation::build. Raw previous
    /// action is intentionally untouched, including the four unexecuted values.
    pub fn project_command(&self, command: &mut Command) {
        if !self.headless() {
            return;
        }
        command.head =
            std::array::from_fn(|i| self.locked_head_rad[i] - DEFAULT_POSITION[HEAD_SLOTS[i]]);
        command.body = BodyPose::default();
        for (value, limit) in command.twist.iter_mut().zip(self.max_twist) {
            *value = if value.is_finite() {
                value.clamp(-limit, limit)
            } else {
                0.0
            };
        }
    }

    /// Allocation-free pack in EXACT physical-ID order. Only the prefix is sent.
    pub fn pack_positions(&self, positions: &[f64; NUM_JOINTS]) -> ([f64; NUM_JOINTS], usize) {
        let mut out = [0.0; NUM_JOINTS];
        let mut count = 0;
        for j in self.active_slots() {
            out[count] = positions[j];
            count += 1;
        }
        (out, count)
    }

    /// Excludes missing slots rather than reporting virtual zero temperatures as
    /// real measurements and diluting the mean. Returns logical joint index.
    pub fn thermal_summary(&self, temps: &[f64; NUM_JOINTS]) -> (usize, f64, f64) {
        let mut hottest = 0;
        let mut maximum = f64::MIN;
        let mut sum = 0.0;
        let mut count = 0;
        for j in self.active_slots() {
            if temps[j] > maximum {
                maximum = temps[j];
                hottest = j;
            }
            sum += temps[j];
            count += 1;
        }
        (hottest, maximum, sum / count as f64)
    }

    pub fn report(&self) -> serde_json::Value {
        serde_json::json!({
            "patch": BUILD_ID,
            "upstream": "a9ec4b2079ef8ee7904014089c885bb07d57d63c",
            "config": self,
            "active_motor_ids": self.motor_ids(),
            "virtual_motor_ids": (0..NUM_JOINTS).filter(|&j| !self.is_present(j)).map(|j| JOINT_IDS[j]).collect::<Vec<_>>(),
            "present_mask": (0..NUM_JOINTS).map(|j| self.is_present(j)).collect::<Vec<_>>(),
            "joint_names": JOINT_NAMES,
            "policy_observation_width": 61,
            "policy_action_width": 14,
            "state_note": "Absent slots are synthetic: fixed position, zero velocity/current, no temperature sensor. Not measured healthy servos.",
            "kinematics_note": "Official geometry is retained; camera/ToF FK is not a calibrated model of a removed or relocated head.",
            "dynamics_validated": false
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::{ACTION_LEN, OBS_LEN, Observation};

    fn headless(mouth: bool) -> Morphology {
        Morphology {
            profile: Profile::Headless,
            mouth_present: mouth,
            ..Default::default()
        }
    }

    #[test]
    fn full_robot_unchanged() {
        let m = Morphology::default();
        assert_eq!(m.motor_ids(), JOINT_IDS.to_vec());
        let mut q = std::array::from_fn(|j| j as f64);
        let old = q;
        m.project_positions(&mut q);
        assert_eq!(old, q);
        assert_eq!(m.pack_positions(&q), (q, NUM_JOINTS));
    }
    #[test]
    fn headless_with_mouth_has_eleven_devices() {
        assert_eq!(
            headless(true).motor_ids(),
            vec![20, 21, 22, 23, 24, 34, 10, 11, 12, 13, 14]
        );
    }
    #[test]
    fn legs_only_has_ten_devices() {
        assert_eq!(
            headless(false).motor_ids(),
            vec![20, 21, 22, 23, 24, 10, 11, 12, 13, 14]
        );
    }
    #[test]
    fn no_leg_can_be_ignored() {
        for mouth in [true, false] {
            let m = headless(mouth);
            for id in [10, 11, 12, 13, 14, 20, 21, 22, 23, 24] {
                assert!(!m.is_absent_id(id));
            }
            for id in [30, 31, 32, 33] {
                assert!(m.is_absent_id(id));
            }
            assert!(!m.is_absent_id(200));
            assert!(!m.is_present(15));
        }
    }
    #[test]
    fn mask_keeps_right_leg_indexes_after_missing_middle_slots() {
        let m = headless(false);
        let q = std::array::from_fn(|j| j as f64 + 0.25);
        let (packed, n) = m.pack_positions(&q);
        assert_eq!(
            &packed[..n],
            &[
                0.25, 1.25, 2.25, 3.25, 4.25, 10.25, 11.25, 12.25, 13.25, 14.25
            ]
        );
    }
    #[test]
    fn virtual_state_is_fixed_not_last_target() {
        let m = headless(false);
        let mut s = Sensors::default();
        s.positions = [9.0; NUM_JOINTS];
        s.velocities = [7.0; NUM_JOINTS];
        s.currents_ma = [42.0; NUM_JOINTS];
        m.project_sensors(&mut s);
        for j in HEAD_SLOTS {
            assert_eq!(s.positions[j], DEFAULT_POSITION[j]);
            assert_eq!(s.velocities[j], 0.0);
            assert_eq!(s.currents_ma[j], 0.0);
        }
        assert_eq!(s.positions[10], 9.0);
        assert_eq!(s.currents_ma[10], 42.0);
    }
    #[test]
    fn home_subtraction_is_done_exactly_once_and_history_is_raw() {
        let m = headless(false);
        let mut s = Sensors::default();
        s.positions = DEFAULT_POSITION;
        m.project_sensors(&mut s);
        let mut c = Command::default();
        m.project_command(&mut c);
        let history = std::array::from_fn::<_, ACTION_LEN, _>(|j| j as f32 * 0.05);
        let o = Observation::build(
            &s.imu,
            &s.positions,
            &s.velocities,
            &DEFAULT_POSITION,
            &history,
            &c,
        );
        assert_eq!(o.as_slice().len(), OBS_LEN);
        assert_eq!(&o.as_slice()[11..15], &[0.0; 4]);
        assert_eq!(&o.as_slice()[34..48], &history);
    }
    #[test]
    fn non_home_lock_generates_measured_offset_and_command_offset() {
        let mut m = headless(true);
        m.locked_head_rad = [0.0; 4];
        let mut s = Sensors::default();
        s.positions = DEFAULT_POSITION;
        m.project_sensors(&mut s);
        let mut c = Command::default();
        m.project_command(&mut c);
        let o = Observation::build(
            &s.imu,
            &s.positions,
            &s.velocities,
            &DEFAULT_POSITION,
            &[0.0; ACTION_LEN],
            &c,
        );
        assert!((o.as_slice()[11] + 0.3491).abs() < 1e-6);
        assert!((o.as_slice()[51] + 0.3491).abs() < 1e-6);
    }
    #[test]
    fn discarded_output_is_not_taken_from_first_ten() {
        let a = std::array::from_fn::<_, ACTION_LEN, _>(|j| j as f32);
        let logical = Observation::scatter_action(&a);
        let (packed, n) = headless(false).pack_positions(&logical);
        assert_eq!(
            &packed[..n],
            &[0.0, 1.0, 2.0, 3.0, 4.0, 9.0, 10.0, 11.0, 12.0, 13.0]
        );
    }
    #[test]
    fn physical_temperature_mean_not_diluted_by_virtual_zeroes() {
        let m = headless(false);
        let mut t = [0.0; NUM_JOINTS];
        for j in m.active_slots() {
            t[j] = 40.0;
        }
        t[12] = 50.0;
        assert_eq!(m.thermal_summary(&t), (12, 50.0, 41.0));
    }
    #[test]
    fn bad_config_rejected() {
        assert!(
            serde_json::from_str::<Morphology>(
                r#"{"schema_version":1,"profile":"headless","ignore_ids":[13]}"#
            )
            .is_err()
        );
        assert!(serde_json::from_str::<Morphology>(r#"{}"#).is_err());
        let mut m = headless(false);
        m.schema_version = 2;
        assert!(m.validate().is_err());
        m.schema_version = 1;
        m.locked_head_rad[0] = f64::NAN;
        assert!(m.validate().is_err());
        m.locked_head_rad = home_head();
        m.max_twist[1] = 0.0;
        assert!(m.validate().is_err());
    }
    #[test]
    fn configured_headless_is_bench_only_without_opt_in() {
        let m: Morphology =
            serde_json::from_str(r#"{"schema_version":1,"profile":"headless"}"#).unwrap();
        assert!(!m.motion_allowed());
        assert!(m.mouth_present);
    }
    #[test]
    fn command_head_is_not_controllable_and_twist_is_limited() {
        let mut c = Command {
            twist: [9.0, -9.0, 9.0],
            head: [1.0; 4],
            body: BodyPose {
                z: 0.03,
                roll: 0.5,
                pitch: 0.5,
            },
        };
        headless(false).project_command(&mut c);
        assert_eq!(c.twist, [0.10, -0.04, 0.40]);
        assert_eq!(c.head, [0.0; 4]);
        assert_eq!(c.body, BodyPose::default());
    }
}
