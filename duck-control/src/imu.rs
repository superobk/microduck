//! The `imu_to_dxl` v2 board (LSM6DSV16X), decoded.
//!
//! One IMU, one code path. The board rides the Dynamixel bus and its 12-byte block is
//! fetched in the same `sync_read` as the servos, so there is no separate sensor to poll
//! and no fusion to run on the host — the chip's SFLP block ships a game-rotation
//! quaternion and estimates its own gyro bias.
//!
//! Block layout at address 124:
//!
//! | bytes | contents |
//! |---|---|
//! | 0..6  | gyro x/y/z, `i16` LE counts, candidate encoding 16.4 counts/dps |
//! | 6..12 | SFLP quaternion x/y/z as IEEE half-precision; `w = √(1 − x² − y² − z²)` |
//!
//! The board's full diagnostic block is 20 bytes (it also carries raw accelerometer, a
//! sample counter and status flags). The control loop consumes only the first 12 so the
//! read fits alongside the servos in one transaction.

use crate::imu_quality::{Config, Mode, Monitor, Report, Status};
use crate::model::NUM_JOINTS;

/// Bytes of the v2 board's block consumed per tick.
pub const IMU_BLOCK_LEN: usize = 12;

/// What the control loop knows about the robot's orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Angular velocity in the trunk frame, rad/s.
    pub gyro: [f64; 3],
    /// Projected gravity in the trunk frame, unit vector. Upright is `[0, 0, -1]`.
    ///
    /// This is what the policy observes, and what fall detection thresholds on.
    pub gravity: [f64; 3],
    /// Orientation, trunk→world, scalar-first `[w, x, y, z]`.
    pub quat: [f64; 4],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            gravity: [0.0, 0.0, -1.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// Candidate wire contract: 16.4 counts per deg/s, converted to rad/s.
/// Use only with that board encoding; this is not the native LSM6DSV16X sensitivity.
const GYRO_RAD_PER_LSB: f64 = (1.0 / 16.4) * std::f64::consts::PI / 180.0;

/// Decodes the board's block into [`ImuData`].
///
/// Stateful only for spike rejection and for holding the last good quaternion — there is
/// no filter here. Keeping the state explicit means [`crate::io::FakeIo`] can drive the
/// same decoder in tests.
pub struct SflpDecoder {
    /// Sensor→trunk mounting rotation, scalar-first.
    mount: [f64; 4],
    /// Last quaternion the board actually produced. Held across blocks that arrive before
    /// SFLP has written its table — snapping to identity would report a robot as upright
    /// when its orientation is simply unknown, which is the worst possible lie to tell
    /// fall detection.
    last_quat: [f64; 4],
    /// Blocks carrying a live quaternion. Gates [`SflpDecoder::ready`].
    quat_samples: u32,
    gyro_history: [[f64; 3]; 2],
    gravity_history: [[f64; 3]; 2],
    monitor: Monitor,
}

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct DecodedSample {
    pub quality_mode: Mode,
    pub gyro_unfiltered: [f64; 3],
    pub measured_quat: Option<[f64; 4]>,
    pub selected_quat: [f64; 4],
    pub gravity_filtered: [f64; 3],
    pub quality: Report,
}

impl Default for SflpDecoder {
    fn default() -> Self {
        Self::new(Self::DEFAULT_MOUNT)
    }
}

impl SflpDecoder {
    /// The `zero3` robot's mount: the board is mounted so that trunk = `[+raw_z, +raw_y, −raw_x]`,
    /// a +90° rotation about Y.
    pub const DEFAULT_MOUNT: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    /// The `beta` robot's mount: the power board lies flat, its axes already the trunk's
    /// (`x` forward, `y` left, `z` up). Read off a beta standing straight (2026-10-06): under
    /// [`Self::DEFAULT_MOUNT`] its trunk read `x` up, a 90° rotation about Y away from this.
    pub const BETA_MOUNT: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    pub fn new(mount: [f64; 4]) -> Self {
        Self {
            mount,
            last_quat: [1.0, 0.0, 0.0, 0.0],
            quat_samples: 0,
            gyro_history: [[0.0; 3]; 2],
            gravity_history: [[0.0, 0.0, -1.0]; 2],
            monitor: Monitor::new(Config::default()),
        }
    }

    /// Whether the chip has demonstrably produced fused output — roughly 0.25 s at 100 Hz.
    ///
    /// Until this is true the orientation is a default, not a measurement. Slice 2's fall
    /// detection must not run before it.
    pub fn ready(&self) -> bool {
        self.quat_samples >= 25 && !self.monitor.faulted()
    }

    pub fn configure_quality(&mut self, config: Config) {
        self.monitor = Monitor::new(config);
    }

    pub fn set_mount(&mut self, mount: [f64; 4]) {
        let config = self.monitor.config;
        *self = Self::new(mount);
        self.configure_quality(config);
    }

    pub fn quality_faulted(&self) -> bool {
        self.monitor.faulted()
    }

    /// A deliberate torque-off ends a failed episode. A good packet alone must not restart it.
    pub fn rearm_after_relax(&mut self) {
        self.monitor.reset();
        self.quat_samples = 0;
        self.gyro_history = [[0.0; 3]; 2];
        self.gravity_history = [[0.0, 0.0, -1.0]; 2];
    }

    pub fn decode(&mut self, raw: &[u8; IMU_BLOCK_LEN]) -> ImuData {
        self.decode_at(raw, duck_ipc_proto::clock::monotonic_ns()).0
    }

    pub fn decode_at(
        &mut self,
        raw: &[u8; IMU_BLOCK_LEN],
        now_ns: u64,
    ) -> (ImuData, DecodedSample) {
        let gyro_sensor = [
            i16::from_le_bytes([raw[0], raw[1]]) as f64 * GYRO_RAD_PER_LSB,
            i16::from_le_bytes([raw[2], raw[3]]) as f64 * GYRO_RAD_PER_LSB,
            i16::from_le_bytes([raw[4], raw[5]]) as f64 * GYRO_RAD_PER_LSB,
        ];
        let gyro = rotate(self.mount, gyro_sensor);

        // All-zero quaternion bytes mean SFLP has not written its table yet — the board
        // just powered up, or its init failed. Keep the last good value.
        let packed = [
            u16::from_le_bytes([raw[6], raw[7]]),
            u16::from_le_bytes([raw[8], raw[9]]),
            u16::from_le_bytes([raw[10], raw[11]]),
        ];
        let mut measured = None;
        if packed != [0, 0, 0] {
            let (x, y, z) = (half(packed[0]), half(packed[1]), half(packed[2]));
            let norm_sq = x * x + y * y + z * z;
            // A quaternion the chip never produced would fail this; ≤ 1.02 allows for
            // half-precision rounding at full scale.
            if x.is_finite() && y.is_finite() && z.is_finite() && norm_sq <= 1.02 {
                let w = (1.0 - norm_sq).max(0.0).sqrt();
                let mount_inv = [
                    self.mount[0],
                    -self.mount[1],
                    -self.mount[2],
                    -self.mount[3],
                ];
                let q = mul([w, x, y, z], mount_inv);
                let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
                if norm > 0.5 {
                    measured = Some([q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]);
                }
            }
        }

        let (predicted, quality) = self.monitor.observe(now_ns, measured, gyro);
        let selected = if self.monitor.config.mode == Mode::Predict {
            predicted
        } else {
            measured
        };
        if let Some(q) = selected {
            self.last_quat = q;
            if measured.is_some()
                && (self.monitor.config.mode == Mode::Shadow || quality.status != Status::Predicted)
            {
                self.quat_samples = self.quat_samples.saturating_add(1);
            }
        }

        // Normalise *before* the median, matching the runtime. Note the consequence: a
        // component-wise median across three unit vectors is not itself unit-norm, so
        // during a transient the policy sees a slightly short vector. Steady state is
        // exact. Whether the training env expects strict unit norm is worth settling when
        // slice 2 wires up observations — normalising after the median would guarantee it,
        // but that is a behaviour change to a path that currently walks.
        let gravity = normalise(rotate_inverse(self.last_quat, [0.0, 0.0, -1.0]));

        let out = ImuData {
            gyro: median3_each(&self.gyro_history, gyro),
            gravity: median3_each(&self.gravity_history, gravity),
            quat: self.last_quat,
        };
        if !self.quality_faulted() {
            self.gyro_history = [self.gyro_history[1], gyro];
            self.gravity_history = [self.gravity_history[1], gravity];
        }
        (
            out,
            DecodedSample {
                quality_mode: self.monitor.config.mode,
                gyro_unfiltered: gyro,
                measured_quat: measured,
                selected_quat: self.last_quat,
                gravity_filtered: out.gravity,
                quality,
            },
        )
    }
}

/// Single-sample spike rejection. A dropped or corrupted block shows up as one wild value;
/// a median over three discards it without the lag of an average.
fn median3_each(history: &[[f64; 3]; 2], now: [f64; 3]) -> [f64; 3] {
    let m = |a: f64, b: f64, c: f64| a.max(b).min(c).max(a.min(b));
    [
        m(history[0][0], history[1][0], now[0]),
        m(history[0][1], history[1][1], now[1]),
        m(history[0][2], history[1][2], now[2]),
    ]
}

/// IEEE 754 binary16 → f64. The LSM6DSV16X ships quaternion components in this format.
fn half(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((bits >> 10) & 0x1F) as i32;
    let frac = (bits & 0x3FF) as f64;
    match exp {
        0 => sign * frac * 2.0_f64.powi(-24),
        0x1F if frac == 0.0 => sign * f64::INFINITY,
        0x1F => f64::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2.0_f64.powi(exp - 15),
    }
}

/// Hamilton product, scalar-first.
pub(crate) fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// `q · v · q⁻¹`
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] + q[0] * t[0] + c[0],
        v[1] + q[0] * t[1] + c[1],
        v[2] + q[0] * t[2] + c[2],
    ]
}

/// `q⁻¹ · v · q` — world vector expressed in the body frame.
pub(crate) fn rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (t, c) = cross_terms(q, v);
    [
        v[0] - q[0] * t[0] + c[0],
        v[1] - q[0] * t[1] + c[1],
        v[2] - q[0] * t[2] + c[2],
    ]
}

fn cross_terms(q: [f64; 4], v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (x, y, z) = (q[1], q[2], q[3]);
    let t = [
        (y * v[2] - z * v[1]) * 2.0,
        (z * v[0] - x * v[2]) * 2.0,
        (x * v[1] - y * v[0]) * 2.0,
    ];
    let c = [
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    ];
    (t, c)
}

/// Unit vector, falling back to "upright" rather than dividing by ~zero.
pub(crate) fn normalise(v: [f64; 3]) -> [f64; 3] {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 0.1 {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    }
}

/// Compile-time assurance that the joint count and this module stay independent — the IMU
/// block is fixed-size regardless of how many servos share the bus.
const _: () = assert!(NUM_JOINTS > 0);

#[cfg(test)]
mod tests {
    use super::*;

    /// Half-precision decoding is hand-rolled, so pin the cases that matter: zero, one,
    /// a negative, and a subnormal. A wrong exponent bias here silently tilts the horizon.
    #[test]
    fn half_precision_decodes_known_values() {
        assert_eq!(half(0x0000), 0.0);
        assert_eq!(half(0x3C00), 1.0);
        assert_eq!(half(0xBC00), -1.0);
        assert_eq!(half(0x3800), 0.5);
        assert!((half(0x3555) - 0.333).abs() < 1e-3);
    }

    /// A block of zeroes is the board saying "SFLP has not started". Reporting identity —
    /// i.e. perfectly upright — would tell fall detection the robot is fine when its
    /// orientation is simply unknown.
    #[test]
    fn all_zero_quaternion_bytes_hold_the_last_good_value() {
        let mut d = SflpDecoder::default();
        assert!(!d.ready());

        let zero = d.decode(&[0u8; IMU_BLOCK_LEN]);
        assert_eq!(zero.quat, [1.0, 0.0, 0.0, 0.0]);
        assert!(!d.ready(), "a zero block must not count as a live sample");

        // 0x3C00 = 1.0 in the x slot would exceed unit norm on its own; use a modest
        // rotation the chip could actually emit.
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes()); // x = 0.125
        let live = d.decode(&block);
        assert_ne!(live.quat, [1.0, 0.0, 0.0, 0.0]);

        let held = d.decode(&[0u8; IMU_BLOCK_LEN]);
        assert_eq!(held.quat, live.quat, "zero block must hold, not reset");
    }

    /// `ready()` gates fall detection in slice 2. If it were true from the first block,
    /// the robot would be judged on a default orientation for the first quarter second.
    #[test]
    fn not_ready_until_the_chip_has_produced_output() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        for _ in 0..24 {
            d.decode(&block);
        }
        assert!(!d.ready());
        d.decode(&block);
        assert!(d.ready());
    }

    /// In steady state gravity must be a unit vector at any orientation — the policy
    /// observes it directly and was trained on normalised input.
    ///
    /// Three identical blocks per orientation so the median settles. Mid-transient the
    /// median blends three different unit vectors component-wise and the result is
    /// slightly short; that is the runtime's behaviour and is deliberately preserved (see
    /// the note in `decode`).
    #[test]
    fn gravity_is_a_unit_vector_in_steady_state() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        for packed in [0x0000u16, 0x3000, 0x3800, 0xB000] {
            block[6..8].copy_from_slice(&packed.to_le_bytes());
            d.decode(&block);
            d.decode(&block);
            let g = d.decode(&block).gravity;
            let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-9,
                "gravity magnitude {mag} for {packed:#06x}"
            );
        }
    }

    /// The transient case, pinned so the bound is known rather than assumed: switching
    /// orientation between ticks blends three vectors and shortens the result. If this
    /// ever gets far from 1.0 the policy is being fed something training never saw.
    #[test]
    fn gravity_stays_close_to_unit_through_a_transient() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x0000u16.to_le_bytes());
        d.decode(&block);
        block[6..8].copy_from_slice(&0x3800u16.to_le_bytes());
        let g = d.decode(&block).gravity;
        let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        assert!(mag > 0.5, "transient gravity collapsed to {mag}");
        assert!(mag <= 1.0 + 1e-9);
    }

    /// Gyro counts are signed. Reading them as unsigned would turn every negative rate
    /// into a large positive one, which reads as a robot spinning.
    #[test]
    fn gyro_counts_are_signed() {
        let mut d = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[0..2].copy_from_slice(&(-1000i16).to_le_bytes());
        // Three identical blocks so the median settles on this sample.
        d.decode(&block);
        d.decode(&block);
        let out = d.decode(&block);
        let magnitude: f64 = out.gyro.iter().map(|v| v.abs()).sum();
        assert!(magnitude > 0.0);
        let expected = 1000.0 * GYRO_RAD_PER_LSB;
        assert!(
            (magnitude - expected).abs() < 1e-9,
            "expected |gyro| {expected}, got {magnitude}"
        );
    }
}

#[cfg(test)]
mod wire_contract_tests {
    use super::*;

    // Fixed physical references catch a wrong divisor, signed decoding or mount axis.
    // Expected values deliberately do not use GYRO_RAD_PER_LSB.
    #[test]
    fn counts_1640_mean_100_dps_on_each_signed_axis() {
        for axis in 0..3 {
            for sign in [-1i16, 1] {
                let mut d = SflpDecoder::default();
                let mut block = [0u8; IMU_BLOCK_LEN];
                block[2 * axis..2 * axis + 2].copy_from_slice(&(sign * 1640).to_le_bytes());
                d.decode(&block);
                d.decode(&block);
                let out = d.decode(&block);
                let mut expected = [0.0f64; 3];
                let (target, direction) = [(2, -1.0), (1, 1.0), (0, 1.0)][axis];
                expected[target] = direction * f64::from(sign) * 100.0;
                for (actual, expected) in out.gyro.iter().zip(expected) {
                    assert!((actual.to_degrees() - expected).abs() < 1e-10);
                }
            }
        }
    }

    // Conversion stays in floating point, including the signed int16 endpoints.
    #[test]
    fn signed_endpoints_have_the_protocol_range() {
        for (counts, expected_dps) in [
            (32767i16, 1997.9878048780488f64),
            (-32768i16, -1998.048780487805f64),
        ] {
            let mut d = SflpDecoder::new(SflpDecoder::BETA_MOUNT);
            let mut block = [0u8; IMU_BLOCK_LEN];
            block[0..2].copy_from_slice(&counts.to_le_bytes());
            d.decode(&block);
            d.decode(&block);
            assert!((d.decode(&block).gyro[0].to_degrees() - expected_dps).abs() < 1e-10);
        }
    }
}
