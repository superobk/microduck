//! Process-isolated CLI/socket tests. No serial device, ONNX or physical robot.
#![cfg(unix)]
use serde_json::{Value, json};
use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

static NEXT: AtomicU64 = AtomicU64::new(0);
fn directory() -> PathBuf {
    // Short enough for macOS's Unix-socket path limit; create_dir refuses collisions.
    let p = PathBuf::from(format!(
        "/tmp/md-hl-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    fs::create_dir(&p).unwrap();
    p
}
struct Robot {
    child: Child,
    root: PathBuf,
}
impl Drop for Robot {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = fs::remove_dir_all(&self.root);
    }
}
impl Robot {
    fn start(mouth: bool, full: bool) -> Self {
        let root = directory();
        fs::write(
            root.join("m.json"),
            serde_json::to_vec(&json!({
                "schema_version":1, "profile":if full {"full15"} else {"headless"},
                "mouth_present":mouth, "allow_motion":full
            }))
            .unwrap(),
        )
        .unwrap();
        fs::write(root.join("robotd.toml"), "[policy]\nenabled=false\n[audio]\nenabled=false\n[safety]\nbattery_empty_shutdown=false\n").unwrap();
        let log = fs::File::create(root.join("log")).unwrap();
        let child = Command::new(env!("CARGO_BIN_EXE_robotd"))
            .args(["--fake", "--no-policy", "--socket"])
            .arg(root.join("s"))
            .arg("--params")
            .arg(root.join("robotd.toml"))
            .env("MICRODUCK_MORPHOLOGY", root.join("m.json"))
            .env("DUCK_RUNTIME_DIR", root.join("runtime"))
            .stdout(Stdio::null())
            .stderr(log)
            .spawn()
            .unwrap();
        let mut robot = Self { child, root };
        let until = Instant::now() + Duration::from_secs(10);
        loop {
            if robot.child.try_wait().unwrap().is_some() {
                panic!(
                    "daemon exited: {}",
                    fs::read_to_string(robot.root.join("log")).unwrap()
                );
            }
            if UnixStream::connect(robot.root.join("s")).is_ok() {
                break;
            }
            assert!(Instant::now() < until, "daemon socket did not appear");
            std::thread::sleep(Duration::from_millis(20));
        }
        robot
    }
    fn call(&self, method: &str, params: Value) -> Value {
        let mut s = UnixStream::connect(self.root.join("s")).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        s.set_write_timeout(Some(Duration::from_secs(3))).unwrap();
        let mut msg =
            serde_json::to_vec(&json!({"jsonrpc":"2.0","id":1,"method":method,"params":params}))
                .unwrap();
        msg.push(b'\n');
        s.write_all(&msg).unwrap();
        let mut line = String::new();
        BufReader::new(s).read_line(&mut line).unwrap();
        serde_json::from_str(&line).unwrap()
    }
}
#[test]
fn both_missing_head_profiles_keep_policy_contract_and_refuse_unsafe_requests() {
    for mouth in [true, false] {
        let r = Robot::start(mouth, false);
        let report = r.call("robot.morphology", json!({}));
        let m = &report["result"];
        assert_eq!(m["patch"], "headless-v015-r2");
        assert_eq!(m["rearm_result"], "not_requested");
        assert_eq!(m["latch_persistent_across_restart"], false);
        // With no fault there is nothing to rearm; do not claim torque is OFF.
        let rearm = r.call("robot.morphology.rearm", json!({}));
        assert_eq!(rearm["result"]["accepted"], false);
        let ids = if mouth {
            json!([20, 21, 22, 23, 24, 34, 10, 11, 12, 13, 14])
        } else {
            json!([20, 21, 22, 23, 24, 10, 11, 12, 13, 14])
        };
        assert_eq!(m["active_motor_ids"], ids);
        assert_eq!(m["policy_observation_width"], 61);
        assert_eq!(m["policy_action_width"], 14);
        for (method, p) in [
            (
                "robot.head",
                json!({"neck_pitch":0.1,"head_pitch":0.1,"head_yaw":0.1,"head_roll":0.1}),
            ),
            ("robot.do", json!({"skill":"roulade"})),
            ("robot.enable", json!({"on":true})),
            ("robot.init", json!({})),
            ("robot.rebootMotors", json!({"ids":[30]})),
        ] {
            let out = r.call(method, p);
            assert_eq!(out["result"]["accepted"], false, "{method}: {out}");
        }
        let out = r.call("robot.mouth", json!({"open":0.5}));
        assert_eq!(out["result"]["accepted"], mouth);
        // Shared IPC intentionally omits an empty built_in field; preserve that
        // wire contract rather than requiring a headless-only schema change.
        let skills = r.call("robot.skills", json!({}));
        assert_eq!(skills["result"]["skills"], json!([]));
        assert!(skills["result"]["built_in"].is_null());
        let sub = r.call("robot.subscribe", json!({"hz":5}));
        assert!(sub["result"]["sitstand"].is_null());
        assert!(sub["result"]["ground_pick"].is_null());
        assert!(sub["result"]["skills"].is_null());
    }
}
#[test]
fn explicit_full15_keeps_existing_head_intent() {
    let r = Robot::start(true, true);
    let m = r.call("robot.morphology", json!({}));
    assert_eq!(
        m["result"]["active_motor_ids"].as_array().unwrap().len(),
        15
    );
    let out = r.call(
        "robot.head",
        json!({"neck_pitch":0.1,"head_pitch":0.1,"head_yaw":0.1,"head_roll":0.1}),
    );
    assert_eq!(out["result"]["accepted"], true);
}
#[test]
fn specified_missing_config_is_fatal_before_bus_open() {
    let root = directory();
    let out = Command::new(env!("CARGO_BIN_EXE_robotd"))
        .args(["--fake", "--no-policy", "--socket"])
        .arg(root.join("s"))
        .env("MICRODUCK_MORPHOLOGY", root.join("missing.json"))
        .env("DUCK_RUNTIME_DIR", root.join("runtime"))
        .output()
        .unwrap();
    assert!(!out.status.success());
    assert!(!root.join("s").exists());
    assert!(String::from_utf8_lossy(&out.stderr).contains("bad morphology"));
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn standalone_init_cannot_bypass_headless_sensor_and_fault_gate() {
    let root = directory();
    fs::write(
        root.join("m.json"),
        r#"{"schema_version":1,"profile":"headless","mouth_present":true,"allow_motion":true}"#,
    )
    .unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_robotd"))
        .args(["--fake", "--socket"])
        .arg(root.join("s"))
        .args(["init", "--duration", "1ms"])
        .env("MICRODUCK_MORPHOLOGY", root.join("m.json"))
        .env("DUCK_RUNTIME_DIR", root.join("runtime"))
        .output()
        .unwrap();
    assert!(!out.status.success());
    assert!(String::from_utf8_lossy(&out.stderr).contains("standalone init refused"));
    assert!(!root.join("s").exists());
    fs::remove_dir_all(root).unwrap();
}
