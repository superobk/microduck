//! Regression inputs inferred from archived host quaternions, NOT UART captures.
//! Gyro is deliberately neutralised to isolate the shape rejection; this does not tune
//! a motion threshold against unavailable pre-filter gyro samples.
use duck_control::imu::SflpDecoder;
use duck_control::imu_quality::{Config, Mode, Status};

#[derive(serde::Deserialize)]
struct Row {
    t_ns: u64,
    raw: [u8; 12],
    expected_quat: [f64; 4],
    expected_gravity: [f64; 3],
    half_shape: bool,
}

#[derive(serde::Deserialize)]
struct Episode {
    rows: Vec<Row>,
    must_latch: bool,
}

fn episodes() -> Vec<Episode> {
    serde_json::from_str(include_str!("fixtures/A-20261019-inferred-windows.json")).unwrap()
}

#[test]
fn shadow_preserves_a_capture_quaternion_and_gravity() {
    for episode in episodes() {
        let mut d = SflpDecoder::default();
        for (i, row) in episode.rows.iter().enumerate() {
            let (out, detail) = d.decode_at(&row.raw, row.t_ns);
            for (actual, expected) in out.quat.iter().zip(row.expected_quat) {
                assert!((actual - expected).abs() < 1e-14);
            }
            if i >= 2 {
                for (actual, expected) in out.gravity.iter().zip(row.expected_gravity) {
                    assert!((actual - expected).abs() < 1e-14);
                }
            }
            assert_eq!(detail.gyro_counts, [0; 3]);
            let sensor = detail.sensor_quat.unwrap();
            for (word, bytes) in detail
                .quaternion_half_words
                .iter()
                .zip(row.raw[6..].as_chunks::<2>().0.iter())
            {
                assert_eq!(*word, u16::from_le_bytes([bytes[0], bytes[1]]));
            }
            assert!(sensor.iter().all(|component| component.is_finite()));
        }
    }
}

#[test]
fn prediction_rejects_all_three_a_half_shape_episodes_before_gravity_filter() {
    for episode in episodes() {
        let mut d = SflpDecoder::default();
        d.configure_quality(Config {
            mode: Mode::Predict,
            ..Default::default()
        });
        let mut before = None;
        let mut rejected = 0;
        for row in &episode.rows {
            let (out, detail) = d.decode_at(&row.raw, row.t_ns);
            if row.half_shape {
                rejected += 1;
                assert!(matches!(
                    detail.quality.status,
                    Status::Predicted | Status::Unknown
                ));
                let previous: [f64; 4] = before.unwrap();
                let dot = out
                    .quat
                    .iter()
                    .zip(previous)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
                    .abs();
                assert!(
                    dot > 0.999,
                    "bad half word reached the selected orientation"
                );
            }
            before = Some(out.quat);
        }
        assert!(rejected > 0);
        assert_eq!(d.quality_faulted(), episode.must_latch);
    }
}
