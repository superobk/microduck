//! Real ID200 payload and read-completion clocks from B's sealed capture.
//! Preserves measured gyro. This is decoder regression evidence, not a walking test.
use duck_control::imu::SflpDecoder;
use duck_control::imu_quality::{Config, Mode, Status};

#[derive(serde::Deserialize)]
struct Row {
    raw: [u8; 12],
    host_read_end_ns: u64,
}

fn rows() -> Vec<Row> {
    include_str!("fixtures/B-20261020-actual-payload-window.jsonl")
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

fn angle(a: [f64; 4], b: [f64; 4]) -> f64 {
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

#[test]
fn real_b_bad_half_sample_is_bridged_and_next_sample_recovers() {
    let mut shadow = SflpDecoder::default();
    let mut protected = SflpDecoder::default();
    protected.configure_quality(Config {
        mode: Mode::Predict,
        ..Config::default()
    });
    let mut previous = None;
    let mut pending_recovery = false;
    let mut bridges = 0;
    for row in rows() {
        let (_, original) = shadow.decode_at(&row.raw, row.host_read_end_ns);
        let (_, fixed) = protected.decode_at(&row.raw, row.host_read_end_ns);
        assert_eq!(original.measured_quat, fixed.measured_quat);
        assert_eq!(original.gyro_unfiltered, fixed.gyro_unfiltered);
        assert_eq!(original.gyro_counts, fixed.gyro_counts);
        if pending_recovery {
            assert_eq!(fixed.quality.status, Status::Good);
            assert_eq!(fixed.selected_quat, fixed.measured_quat.unwrap());
            pending_recovery = false;
        }
        if fixed.quality.status == Status::Predicted {
            bridges += 1;
            assert_eq!(fixed.quaternion_half_words[0], 0xb400);
            assert_eq!(fixed.gyro_counts, [55, -274, 21]);
            assert!(angle(previous.unwrap(), original.selected_quat) > 36.0);
            assert!(angle(previous.unwrap(), fixed.selected_quat) < 0.4);
            assert!(angle(fixed.selected_quat, [1.0, 0.0, 0.0, 0.0]) > 60.0);
            pending_recovery = true;
        }
        assert!(!protected.quality_faulted());
        previous = Some(fixed.selected_quat);
    }
    assert_eq!(bridges, 1);
    assert!(!pending_recovery);
}

#[test]
fn real_static_640ms_episode_is_bridged_only_with_continuous_gyro() {
    let rows: Vec<Row> = include_str!("fixtures/guard-20261020-static-half-640ms.jsonl")
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    let mut decoder = SflpDecoder::default();
    decoder.configure_quality(Config {
        mode: Mode::Predict,
        max_predict_ns: 1_000_000_000,
        ..Config::default()
    });
    let mut bridged = 0;
    let mut previous = None;
    for row in &rows {
        let (_, detail) = decoder.decode_at(&row.raw, row.host_read_end_ns);
        assert!(!decoder.quality_faulted());
        if detail.quaternion_half_words[0] == 0x3400 {
            bridged += 1;
            assert_eq!(detail.quality.status, Status::Predicted);
            assert!(angle(previous.unwrap(), detail.selected_quat) < 0.1);
            assert!(angle(detail.selected_quat, detail.measured_quat.unwrap()) > 30.0);
        }
        previous = Some(detail.selected_quat);
    }
    assert_eq!(bridged, 33);
    assert!(decoder.ready());
    let last = rows.last().unwrap();
    let (_, detail) = decoder.decode_at(&last.raw, last.host_read_end_ns + 20_000_000);
    assert_eq!(detail.quality.status, Status::Good);
    assert_eq!(detail.selected_quat, detail.measured_quat.unwrap());
}

#[test]
fn startup_pause_restarts_warmup_but_a_ready_decoder_does_not_hide_a_gap() {
    let raw = rows()[0].raw;
    let mut decoder = SflpDecoder::default();
    decoder.configure_quality(Config {
        mode: Mode::Predict,
        max_predict_ns: 1_000_000_000,
        ..Config::default()
    });
    decoder.decode_at(&raw, 1);
    let (_, report) = decoder.decode_at(&raw, 233_000_001);
    assert_eq!(report.quality.status, Status::Bootstrap);
    assert!(!decoder.ready());
    for i in 1..25 {
        decoder.decode_at(&raw, 233_000_001 + i * 20_000_000);
    }
    assert!(decoder.ready());
    let (_, report) = decoder.decode_at(&raw, 800_000_001);
    assert_eq!(report.quality.reason, "time_gap");
    assert!(decoder.quality_faulted());
    decoder.decode_at(&raw, 1_000_000_001);
    assert!(
        decoder.quality_faulted(),
        "a gap must not clear a latched fault"
    );
    decoder.rearm_after_relax();
    assert!(!decoder.ready());
    assert_eq!(
        decoder.decode_at(&raw, 1_020_000_001).1.quality.status,
        Status::Bootstrap
    );
}

#[test]
fn real_heading_reference_episode_does_not_stop_valid_tilt_updates() {
    let mut decoder = SflpDecoder::default();
    decoder.configure_quality(Config {
        mode: Mode::Predict,
        max_predict_ns: 1_000_000_000,
        ..Config::default()
    });
    let mut aligned = 0;
    for line in include_str!("fixtures/guard-20261020-heading-reference.jsonl").lines() {
        let row: Row = serde_json::from_str(line).unwrap();
        let (_, detail) = decoder.decode_at(&row.raw, row.host_read_end_ns);
        assert!(!decoder.quality_faulted());
        if detail.quality.status == Status::HeadingAligned {
            aligned += 1;
        }
    }
    assert!(aligned > 50);
    assert!(decoder.ready());
}
