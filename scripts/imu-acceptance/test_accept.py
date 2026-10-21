import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import accept


def rows(fault=False):
    return [dict(host_read_end_ns=i * 20_000_000, dropped_records=0,
                 decoded=dict(quality_mode="predict",
                              quality=dict(status="unknown" if fault and i >= 400 else "good",
                                           since_good_ms=0),
                              selected_valid=not (fault and i >= 400),
                              control_ready=not (fault and i >= 400))) for i in range(500)]


def state(stage):
    return dict(params=dict(policy="held" if stage == "bench" else "walk", safety=dict(limp=False,
                fallen=False, picked_up=False), move=dict(requested=[0., 0., 0.], applied=[0., 0., 0.])))


def evidence():
    return dict(release_confirmed=True, journal_checked=True, torque_on_ns=[], reset_ns=[])


class AcceptanceTests(unittest.TestCase):
    def assess(self, values=None, stage="bench", physical=True, complete=True, states=None):
        return accept.assess(rows(True) if values is None else values, stage,
                             states or [state(stage)], physical, complete, evidence())

    def test_limp_false_is_normal_after_relax(self):
        self.assertFalse(state("bench")["params"]["safety"]["limp"])
        self.assertEqual(self.assess()["verdict"], "PASS")

    def test_release_evidence_is_required_even_if_limp_true(self):
        s = state("bench")
        s["params"]["safety"]["limp"] = True
        result = accept.assess(rows(True), "bench", [s], True, True)
        self.assertEqual(result["verdict"], "FAIL")

    def test_torque_on_invalidates_bench(self):
        e = evidence()
        e["torque_on_ns"] = [1]
        self.assertEqual(accept.assess(rows(True), "bench", [state("bench")],
                                      True, True, e)["verdict"], "FAIL")

    def test_external_reset_is_interruption_not_uncommanded_recovery(self):
        data = rows(True)
        data[-1] = rows()[-1]
        e = evidence()
        e["reset_ns"] = [data[-1]["host_read_end_ns"] - 1]
        result = accept.assess(data, "bench", [state("bench")], True, True, e)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertNotIn("未经 relax 自动恢复", result["errors"])
        self.assertTrue(any("验收被中断" in error for error in result["errors"]))

    def test_journal_window_excludes_final_cleanup_but_detects_midrun_reset(self):
        def entry(ns, message):
            return dict(__MONOTONIC_TIMESTAMP=str(ns // 1000), MESSAGE=message)
        entries = [entry(9_000_000_000, "robot.relax: torque off"),
                   entry(16_000_000_000, "IMU quality monitor rearmed"),
                   entry(16_010_000_000, "robot.relax: torque off"),
                   entry(21_000_000_000, "robot.relax: torque off")]
        e = accept.journal_evidence(entries, dict(monotonic_ns=10_000_000_000),
                                    11_000_000_000, 20_000_000_000)
        self.assertTrue(e["release_confirmed"])
        self.assertEqual(e["reset_ns"], [16_000_000_000, 16_010_000_000])
        self.assertEqual(e["torque_on_ns"], [])
        e = accept.journal_evidence(entries[1:], dict(monotonic_ns=10_000_000_000),
                                    11_000_000_000, 20_000_000_000)
        self.assertFalse(e["journal_checked"])

    def test_missing_or_corrupt_prerequisite_gives_next_command(self):
        with tempfile.TemporaryDirectory() as d, patch.object(accept, "BASE", Path(d)):
            for content in (None, "{broken"):
                if content:
                    (Path(d) / "bench-pass.json").write_text(content)
                with self.assertRaisesRegex(RuntimeError, "--stage bench --long"):
                    accept.prerequisite("stand", {})
            accept.prerequisite("bench", {})

    def test_prerequisite_still_requires_same_identity(self):
        with tempfile.TemporaryDirectory() as d, patch.object(accept, "BASE", Path(d)):
            snapshot = dict(robotd_sha256="a")
            accept.save(Path(d) / "bench-pass.json",
                        dict(verdict="PASS", identity=accept.identity(snapshot)))
            accept.prerequisite("stand", snapshot)
            with self.assertRaisesRegex(RuntimeError, "身份改变"):
                accept.prerequisite("stand", dict(robotd_sha256="b"))

    def test_watchdog_cancel_depends_on_release_not_experiment_success(self):
        with patch.object(accept, "run") as run:
            accept.cancel_watchdog("imu-accept-stop-123", False)
            run.assert_not_called()
            accept.cancel_watchdog("imu-accept-stop-123", True)
            run.assert_called_once_with("systemctl", "stop", "imu-accept-stop-123.timer",
                                        "imu-accept-stop-123.service")

    def test_only_owned_stale_timers_and_services_are_cleared(self):
        output = ("imu-accept-stop-123.timer loaded active waiting\n"
                  "imu-accept-stop-123.service loaded inactive dead\n"
                  "unrelated.timer loaded active waiting\n"
                  "imu-accept-stop-bogus.timer loaded active waiting")
        with tempfile.TemporaryDirectory() as d, \
             patch.object(accept, "run", return_value=output), \
             patch.object(Path, "exists", return_value=False), \
             patch.object(accept, "cancel_watchdog") as cancel:
            accept.clear_stale_watchdogs(Path(d))
            cancel.assert_called_once_with("imu-accept-stop-123", True)

    def test_live_old_session_blocks_stale_timer_cleanup(self):
        with tempfile.TemporaryDirectory() as d, \
             patch.object(accept, "run", return_value="imu-accept-stop-123.timer loaded active waiting"), \
             patch.object(Path, "exists", return_value=True), \
             patch.object(accept, "cancel_watchdog") as cancel:
            with self.assertRaisesRegex(RuntimeError, "另一轮"):
                accept.clear_stale_watchdogs(Path(d))
            cancel.assert_not_called()

    def test_aborted_session_cancels_timer_only_after_successful_final_relax(self):
        # Reproduce the first real bench's user-stop path, including finally cleanup.
        for cleanup_fails in (False, True):
            with self.subTest(cleanup_fails=cleanup_fails), tempfile.TemporaryDirectory() as d:
                base = Path(d)
                (base / "imu-1.jsonl").write_text("")
                original_read = Path.read_text

                def read(path, *args, **kwargs):
                    return "test-machine" if str(path) == "/etc/machine-id" else original_read(path, *args, **kwargs)

                def command(*args):
                    if "-p" in args and "Environment" in args:
                        return ("ROBOT_IMU_MODE=predict ROBOT_IMU_MAX_PREDICT_MS=40 "
                                "ROBOT_IMU_TOLERANCE_DEG=5 ROBOT_IMU_TRACE_DIR=" + d)
                    return ""

                def collect(*args, **kwargs):
                    (base / "bench-test" / "state.jsonl").write_text("{}\n")
                    return Mock()

                ack = dict(monotonic_ns=1, cursor="test")
                releases = [ack, ack, RuntimeError("no fresh release") if cleanup_fails else ack]
                with patch.object(accept, "BASE", base), patch.object(accept, "TRACE", base), \
                     patch.object(accept, "TRACE_FILE", None), patch.object(accept, "AUDIT_DIR", None), \
                     patch.object(accept, "RUN_DEADLINE", float("inf")), \
                     patch.object(Path, "read_text", read), patch.object(accept, "run", side_effect=command), \
                     patch.object(accept, "rpc") as rpc, patch.object(accept, "relax", side_effect=releases), \
                     patch.object(accept, "clear_stale_watchdogs"), patch.object(accept, "cancel_watchdog") as cancel, \
                     patch.object(accept.capture, "snapshot", return_value=dict(policy_identity_issues=[])), \
                     patch.object(accept.shutil, "which", return_value="/bin/robotctl"), \
                     patch.object(accept.subprocess, "Popen", side_effect=collect), \
                     patch.object(accept.time, "sleep"), patch.object(accept, "phase", side_effect=RuntimeError("用户停止")):
                    self.assertEqual(accept.session("bench", True, "bench-test", "sha"), 2)
                    self.assertEqual(cancel.call_args.args[1], not cleanup_fails)
                    self.assertNotIn("robot.init", [c.args[0] for c in rpc.call_args_list])
                    self.assertNotIn("robot.enable", [c.args[0] for c in rpc.call_args_list])
                    self.assertTrue((base / "bench-test" / "finished-acceptance.json").exists())
                    self.assertFalse((base / "bench-pass.json").exists())

    def test_no_fault_is_not_fault_response_pass(self):
        self.assertEqual(self.assess(rows())["verdict"], "NOT_EXERCISED")

    def test_latched_fault_with_invalid_output_passes_supported_scope(self):
        result = self.assess()
        self.assertEqual(result["verdict"], "PASS")
        self.assertEqual(result["scope"], "supported_torque_off_guard")

    def test_no_automatic_resume(self):
        data = rows(True)
        data[-1] = rows()[-1]
        self.assertEqual(self.assess(data)["verdict"], "FAIL")

    def test_invalid_sample_never_ready(self):
        data = rows(True)
        data[-1]["decoded"]["control_ready"] = True
        self.assertEqual(self.assess(data)["verdict"], "FAIL")

    def test_bounded_prediction(self):
        data = rows()
        data[200]["decoded"]["quality"] = dict(status="predicted", since_good_ms=40.01)
        self.assertEqual(self.assess(data)["verdict"], "FAIL")

    def test_loss_shadow_and_missing_flags_fail(self):
        for change in ("loss", "shadow", "flag"):
            data = rows(True)
            if change == "loss":
                data[0]["dropped_records"] = 1
            elif change == "shadow":
                data[0]["decoded"]["quality_mode"] = "shadow"
            else:
                del data[0]["decoded"]["selected_valid"]
            self.assertEqual(self.assess(data)["verdict"], "FAIL")

    def test_incomplete_or_physical_failure_never_passes(self):
        self.assertEqual(self.assess(physical=False)["verdict"], "FAIL")
        self.assertEqual(self.assess(complete=False)["verdict"], "FAIL")
        self.assertEqual(self.assess([])["verdict"], "FAIL")

    def test_powered_fault_blocks_next_stage(self):
        self.assertEqual(self.assess(stage="stand")["verdict"], "FAIL")

    def test_zero_stand_rejects_intent_and_nonrunning_policy(self):
        s = state("stand")
        s["params"]["move"]["requested"][0] = -.4
        self.assertEqual(self.assess(rows(), "stand", states=[s])["verdict"], "FAIL")
        s = state("stand")
        s["params"]["policy"] = "held"
        self.assertEqual(self.assess(rows(), "stand", states=[s])["verdict"], "FAIL")

    def test_zero_stand_can_pass(self):
        self.assertEqual(self.assess(rows(), "stand")["verdict"], "PASS")

    def test_walk_requires_actual_bounded_forward_intent(self):
        self.assertEqual(self.assess(rows(), "walk")["verdict"], "FAIL")
        s = state("walk")
        for k in ("requested", "applied"):
            s["params"]["move"][k][0] = .05
        self.assertEqual(self.assess(rows(), "walk", states=[s])["verdict"], "PASS")
        s["params"]["move"]["applied"][0] = -.1
        self.assertEqual(self.assess(rows(), "walk", states=[s])["verdict"], "FAIL")

    def test_confirm_timeout_does_not_read_or_enable(self):
        with patch.object(accept.select, "select", return_value=([], [], [])):
            with self.assertRaises(RuntimeError):
                accept.confirm("test")

    def test_deadline_blocks_motion_but_not_stop(self):
        with patch.object(accept, "RUN_DEADLINE", -1):
            for method in ("robot.init", "robot.enable", "robot.move"):
                with self.assertRaisesRegex(RuntimeError, "超时"):
                    accept.rpc(method)

    def test_relax_requires_fresh_ack_not_only_rpc(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(accept, "rpc", return_value={"accepted": True}), \
                 patch.object(accept, "run", return_value="-- cursor: fake"), \
                 patch.object(accept.time, "monotonic", side_effect=[0, 11]):
                with self.assertRaisesRegex(RuntimeError, "torque off"):
                    accept.relax(Path(d))
                self.assertFalse((Path(d) / "relax-confirmed.json").exists())

    def test_identity_changes_detected(self):
        a = dict(robotd_sha256="a", config_sha256="b", models_sha256={"walk": "c"}, machine_id="d")
        b = copy.deepcopy(a)
        b["models_sha256"]["walk"] = "new"
        self.assertNotEqual(accept.identity(a), accept.identity(b))

    def test_ready_does_not_allow_grossly_wrong_initial_tilt(self):
        row = rows()[0]
        row["decoded"]["selected_quat"] = [0., 1., 0., 0.]
        with self.assertRaisesRegex(RuntimeError, "15°"):
            accept.require_upright(row)
        row["decoded"]["selected_quat"] = [1., 0., 0., 0.]
        accept.require_upright(row)
        row["decoded"]["quality"]["status"] = "predicted"
        with self.assertRaises(RuntimeError):
            accept.require_upright(row)

    def test_setup_gate_ignores_quaternion_sign_and_pure_yaw(self):
        row = rows()[0]
        row["decoded"]["selected_quat"] = [0., 0., 0., -1.]
        accept.require_upright(row)
        row["decoded"]["selected_quat"] = [float("nan"), 0., 0., 0.]
        with self.assertRaises(RuntimeError):
            accept.require_upright(row)


if __name__ == "__main__":
    unittest.main()
