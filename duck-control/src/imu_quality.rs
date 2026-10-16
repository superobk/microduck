//! Host-side consistency checks. Shadow mode never changes the decoder's output.
//!
//! Predict mode is an explicit bench experiment, not an identification of which sensor
//! is wrong. It bridges a short disagreement and latches unknown rather than inventing
//! an upright pose. Only an explicit torque-off can rearm a latched fault.

use crate::imu::{mul, normalise, rotate_inverse};

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Shadow,
    Predict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Bootstrap,
    Good,
    Predicted,
    Unknown,
}

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Report {
    pub status: Status,
    pub reason: &'static str,
    pub gravity_residual_deg: Option<f64>,
    pub rotation_residual_deg: Option<f64>,
    pub threshold_deg: f64,
    pub since_good_ms: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub mode: Mode,
    pub tolerance_deg: f64,
    pub max_predict_ns: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Shadow,
            tolerance_deg: 5.0,
            max_predict_ns: 40_000_000,
        }
    }
}

pub struct Monitor {
    pub config: Config,
    previous: Option<([f64; 4], [f64; 3], u64)>,
    good_at: u64,
    faulted: bool,
}

impl Monitor {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            previous: None,
            good_at: 0,
            faulted: false,
        }
    }

    pub fn faulted(&self) -> bool {
        self.config.mode == Mode::Predict && self.faulted
    }

    pub fn reset(&mut self) {
        *self = Self::new(self.config);
    }

    pub fn observe(
        &mut self,
        now: u64,
        measured: Option<[f64; 4]>,
        gyro: [f64; 3],
    ) -> (Option<[f64; 4]>, Report) {
        let mut report = Report {
            status: Status::Unknown,
            reason: "uninitialised",
            gravity_residual_deg: None,
            rotation_residual_deg: None,
            threshold_deg: self.config.tolerance_deg,
            since_good_ms: 0.0,
        };
        let measured = measured.filter(|q| {
            q.iter().all(|v| v.is_finite())
                && (q.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 0.001
        });
        if self.faulted() {
            report.reason = "latched_fault";
            return (None, report);
        }
        if !gyro.iter().all(|v| v.is_finite()) {
            self.faulted = true;
            report.reason = "invalid_gyro";
            return (None, report);
        }
        let Some((previous, previous_gyro, previous_at)) = self.previous else {
            if let Some(q) = measured {
                self.previous = Some((q, gyro, now));
                self.good_at = now;
                report.status = Status::Bootstrap;
                report.reason = "first_measurement";
            }
            return (measured, report);
        };
        let elapsed = now.checked_sub(previous_at).filter(|ns| *ns > 0);
        let age = now.saturating_sub(self.good_at);
        report.since_good_ms = age as f64 / 1e6;
        // The read clock is not a chip clock. A missing interval cannot be filled with
        // whichever rate happened to arrive next, even if the endpoint looks plausible.
        let Some(elapsed) = elapsed.filter(|ns| *ns <= self.config.max_predict_ns) else {
            self.faulted = true;
            report.reason = "time_gap";
            if self.config.mode == Mode::Shadow {
                self.previous = measured.map(|q| (q, gyro, now));
                self.good_at = now;
            }
            return (None, report);
        };
        let dt = elapsed as f64 / 1e9;
        let omega = std::array::from_fn(|i| (previous_gyro[i] + gyro[i]) / 2.0);
        let predicted = integrate(previous, omega, dt);
        // Initial bench allowance for one tick of unknown gyro/quaternion alignment.
        // These numbers are configurable and have NOT been validated on a standing robot.
        report.threshold_deg += norm(gyro).max(norm(previous_gyro)).to_degrees() * 0.020;
        if let Some(q) = measured {
            report.gravity_residual_deg = Some(vector_angle(gravity(predicted), gravity(q)));
            report.rotation_residual_deg = Some(rotation_angle(predicted, q));
            if report
                .gravity_residual_deg
                .unwrap()
                .max(report.rotation_residual_deg.unwrap())
                <= report.threshold_deg
            {
                self.previous = Some((q, gyro, now));
                self.good_at = now;
                report.status = Status::Good;
                report.reason = "consistent";
                return (Some(q), report);
            }
        }
        if age <= self.config.max_predict_ns {
            self.previous = Some((predicted, gyro, now));
            report.status = Status::Predicted;
            report.reason = if measured.is_some() {
                "innovation"
            } else {
                "invalid_quaternion"
            };
            (Some(predicted), report)
        } else {
            self.faulted = true;
            report.reason = "prediction_expired";
            if self.config.mode == Mode::Shadow {
                self.previous = measured.map(|q| (q, gyro, now));
                self.good_at = now;
            }
            (None, report)
        }
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

pub(crate) fn gravity(q: [f64; 4]) -> [f64; 3] {
    normalise(rotate_inverse(q, [0.0, 0.0, -1.0]))
}

fn vector_angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>() / norm(a) / norm(b))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

fn rotation_angle(a: [f64; 4], b: [f64; 4]) -> f64 {
    2.0 * a
        .iter()
        .zip(b)
        .map(|(x, y)| x * y)
        .sum::<f64>()
        .abs()
        .clamp(0.0, 1.0)
        .acos()
        .to_degrees()
}

fn integrate(q: [f64; 4], omega: [f64; 3], dt: f64) -> [f64; 4] {
    let speed = norm(omega);
    if speed < 1e-12 {
        return q;
    }
    let half = speed * dt / 2.0;
    let scale = half.sin() / speed;
    let predicted = mul(
        q,
        [
            half.cos(),
            omega[0] * scale,
            omega[1] * scale,
            omega[2] * scale,
        ],
    );
    let length = predicted.iter().map(|v| v * v).sum::<f64>().sqrt();
    predicted.map(|v| v / length)
}

#[cfg(test)]
mod tests {
    use super::*;
    const UPRIGHT: [f64; 4] = [1.0, 0.0, 0.0, 0.0];
    fn guard() -> Monitor {
        Monitor::new(Config {
            mode: Mode::Predict,
            ..Config::default()
        })
    }
    fn pitch(theta: f64) -> [f64; 4] {
        [(theta / 2.0).cos(), 0.0, (theta / 2.0).sin(), 0.0]
    }

    #[test]
    fn genuine_fast_hand_rotation_is_accepted() {
        let mut m = guard();
        let rate = 25_f64.to_radians() / 0.020;
        m.observe(1, Some(UPRIGHT), [0.0, rate, 0.0]);
        let (_, r) = m.observe(
            20_000_001,
            Some(pitch(25_f64.to_radians())),
            [0.0, rate, 0.0],
        );
        assert_eq!(r.status, Status::Good);
        assert!(r.gravity_residual_deg.unwrap() < 1e-5);
    }

    #[test]
    fn bad_measurement_is_bridged_then_fault_latches_from_last_good_time() {
        let mut m = guard();
        m.observe(1, Some(UPRIGHT), [0.0; 3]);
        for t in [20_000_001, 40_000_001] {
            let (q, r) = m.observe(t, Some(pitch(0.6)), [0.0; 3]);
            assert_eq!(r.status, Status::Predicted);
            assert_eq!(q, Some(UPRIGHT));
        }
        assert_eq!(
            m.observe(60_000_001, Some(pitch(0.6)), [0.0; 3]).1.status,
            Status::Unknown
        );
        assert!(m.faulted());
        assert_eq!(m.observe(80_000_001, Some(UPRIGHT), [0.0; 3]).0, None);
        m.reset();
        assert_eq!(
            m.observe(100_000_001, Some(UPRIGHT), [0.0; 3]).1.status,
            Status::Bootstrap
        );
    }

    #[test]
    fn good_return_after_one_bad_tick_does_not_reject_recovery() {
        let mut m = guard();
        m.observe(1, Some(UPRIGHT), [0.0; 3]);
        m.observe(20_000_001, Some(pitch(0.6)), [0.0; 3]);
        assert_eq!(
            m.observe(40_000_001, Some(UPRIGHT), [0.0; 3]).1.status,
            Status::Good
        );
    }

    #[test]
    fn quaternion_sign_is_not_motion() {
        let mut m = guard();
        m.observe(1, Some(UPRIGHT), [0.0; 3]);
        assert_eq!(
            m.observe(20_000_001, Some([-1.0, 0.0, 0.0, 0.0]), [0.0; 3])
                .1
                .status,
            Status::Good
        );
    }

    #[test]
    fn a_real_flip_crossing_the_quaternion_hemisphere_is_not_rejected() {
        let mut m = guard();
        let rate = 0.3 / 0.020;
        m.observe(1, Some(pitch(3.0)), [0.0, rate, 0.0]);
        let q = pitch(3.3).map(|v| -v);
        let (selected, r) = m.observe(20_000_001, Some(q), [0.0, rate, 0.0]);
        assert_eq!(r.status, Status::Good);
        assert_eq!(selected, Some(q));
    }

    #[test]
    fn long_gap_or_invalid_gyro_cannot_be_filled_with_identity() {
        let mut m = guard();
        m.observe(1, Some(pitch(0.2)), [0.0; 3]);
        assert_eq!(m.observe(100_000_001, Some(UPRIGHT), [0.0; 3]).0, None);
        assert!(m.faulted());
        let mut m = guard();
        m.observe(1, Some(UPRIGHT), [0.0; 3]);
        assert_eq!(
            m.observe(20_000_001, Some(UPRIGHT), [f64::NAN, 0.0, 0.0]).0,
            None
        );
        assert!(m.faulted());
    }

    #[test]
    fn missing_initial_measurement_is_unknown() {
        let mut m = guard();
        assert_eq!(m.observe(1, None, [0.0; 3]).0, None);
        assert_eq!(
            m.observe(20_000_001, None, [0.0; 3]).1.status,
            Status::Unknown
        );
    }
}
