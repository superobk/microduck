import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import accept


def rows(fault=False):
    return [dict(host_read_end_ns=i * 20_000_000, dropped_records=0,
                 decoded=dict(quality_mode="predict",
                              quality=dict(status="unknown" if fault and i >= 400 else "good",
                                           since_good_ms=0),
                              selected_valid=not (fault and i >= 400),
                              control_ready=not (fault and i >= 400))) for i in range(500)]


def state(stage):
    return dict(params=dict(policy="walk", safety=dict(limp=stage == "bench",
                fallen=False, picked_up=False), move=dict(requested=[0., 0., 0.], applied=[0., 0., 0.])))


class AcceptanceTests(unittest.TestCase):
    def assess(self, values=None, stage="bench", physical=True, complete=True, states=None):
        return accept.assess(rows(True) if values is None else values, stage,
                             states or [state(stage)], physical, complete)

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
