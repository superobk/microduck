#!/usr/bin/env python3
"""Replay sealed complete captures. No network, robot IO, deployment or input repair."""
import collections
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import subprocess

import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--data-root", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--executable", type=Path, required=True)
args = parser.parse_args()
ROOT, OUT, EXE = args.data_root.resolve(), args.output.resolve(), args.executable.resolve()
CAPTURES = {
    "A": ("logs/imu-hardware-comparison/A-20261020T065630Z-65459/imu-payload-capture.jsonl",
          "150b218d9cf949c293889626392aa320beda1d174b70152977b128426763a3c0", 22415),
    "B": ("logs/imu-hardware-comparison/B-20261020T073323Z-71466/imu-payload-capture.jsonl",
          "6075fa21baf5907f8fb6d10d83ef4253839fe0d8aa5d556b31caa6e9559f9507", 22385),
    "static_640ms": ("logs/imu-guard-live-20261020T084326Z/fault-predict-full.jsonl",
                     "3fa304f53a87adb6999a8962f3dea11d358181451113cd19a63e79fc54182e63", 15087),
    "short_healthy_full": ("logs/imu-protect-live-20261020T090116Z/imu-payload-full.jsonl",
                          "0587edf89698081d1ec8881628d09a7892e673f110835b31dae1ed8df56b3afe", 7566),
    "long_inconsistent": ("logs/imu-protect-active-20261020T090626Z/heading-fault.jsonl",
                          "b5ae66b084f7044dbe69884067c2a4bf9e92f23677c13c5d27a4c71aed279c53", 35935),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def angle(a, b):
    dot = sum(x*y for x, y in zip(a, b))
    return math.degrees(2 * math.acos(min(1, abs(dot))))


def gravity(q):
    w, x, y, z = q
    return [2*(w*y-x*z), -2*(y*z+w*x), 2*(x*x+y*y)-1]


def run(name, relative, sha, count, budget):
    source = ROOT / relative
    assert digest(source) == sha, f"{name}: source checksum mismatch"
    command = [str(EXE), "--predict"]
    if budget == 1000:
        command += ["--bridge-1000ms"]
    statuses, reasons = collections.Counter(), collections.Counter()
    result = dict(source=relative, sha256=sha, expected_records=count, budget_ms=budget)
    records, first_unknown, previous, first_at, last_at = 0, None, None, None, None
    max_prediction_step, max_alignment_gravity_error = 0.0, 0.0
    faults = []
    replay_path = OUT / f"{name}-{budget}ms.jsonl.gz"
    with source.open("rb") as stdin, source.open() as original, gzip.open(replay_path, "wt") as saved:
        process = subprocess.Popen(command, stdin=stdin, stdout=subprocess.PIPE, text=True)
        assert process.stdout is not None
        try:
            for index, (line, output) in enumerate(itertools.zip_longest(original, process.stdout), 1):
                assert line is not None and output is not None, f"{name}: truncated replay"
                row, replay = json.loads(line), json.loads(output)
                assert row["device_id"] == 200 and row["register_address"] == 124
                assert row["dropped_records"] == 0, f"{name}: trace loss"
                assert row["raw"] == replay["raw"]
                assert row["host_read_end_ns"] == replay["host_read_end_ns"]
                d, at = replay["decoded"], replay["host_read_end_ns"]
                raw = bytes(row["raw"])
                # Independent IEEE half reference; neither W nor mount maths produces XYZ.
                xyz = struct.unpack("<eee", raw[6:])
                assert d["sensor_quat"][1:] == list(xyz)
                assert d["gyro_counts"] == list(struct.unpack("<hhh", raw[:6]))
                # Original recorded measured quaternion was decoded before any old guard.
                assert max(abs(a-b) for a, b in zip(
                    d["measured_quat"], row["decoded"]["measured_quat"])) < 1e-12
                quality = d["quality"]
                status, reason = quality["status"], quality["reason"]
                statuses[status] += 1
                reasons[reason] += 1
                first_at = at if first_at is None else first_at
                last_at = at
                if first_unknown is not None:
                    assert status == "unknown", "latched failure resumed without explicit relax"
                if status == "unknown":
                    assert not d["selected_valid"] and not d["control_ready"]
                    if first_unknown is None:
                        first_unknown = index
                        faults.append(dict(row=index, host_read_end_ns=at, quality=quality))
                else:
                    assert d["selected_valid"]
                if status == "predicted":
                    assert quality["since_good_ms"] <= budget
                    assert previous is not None
                    assert 0 < at - previous[0] <= 40_000_000
                    max_prediction_step = max(max_prediction_step, angle(previous[1], d["selected_quat"]))
                if status == "heading_aligned":
                    delta = max(abs(a-b) for a, b in zip(
                        gravity(d["selected_quat"]), gravity(d["measured_quat"])))
                    max_alignment_gravity_error = max(max_alignment_gravity_error, delta)
                    assert delta < 1e-12, "heading correction changed measured tilt"
                    assert quality["gravity_residual_deg"] <= quality["threshold_deg"]
                previous = (at, d["selected_quat"])
                records += 1
                saved.write(output)
            assert process.wait() == 0
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    assert records == count
    # Predeclared outcomes distinguish protection from continuity/recovery.
    if name in ("A", "short_healthy_full"):
        assert first_unknown is None and statuses["predicted"] == 0
    elif name == "B":
        assert first_unknown is None and statuses["predicted"] == 1
    elif name == "static_640ms" and budget == 1000:
        assert first_unknown is None and statuses["predicted"] == 33
    else:
        assert first_unknown is not None
        assert faults[0]["quality"]["reason"] == "prediction_expired"
        assert faults[0]["quality"]["since_good_ms"] > budget
        assert faults[0]["quality"]["since_good_ms"] <= budget + 40
    result.update(records=records, duration_s=(last_at-first_at)/1e9,
                  statuses=dict(statuses), reasons=dict(reasons), first_fault=faults,
                  max_predicted_step_deg=max_prediction_step,
                  max_heading_gravity_component_error=max_alignment_gravity_error,
                  outcome="PROTECTED_LOCKOUT" if first_unknown else "CONTINUITY_PASS",
                  replay=replay_path.name, replay_sha256=digest(replay_path))
    print(f"{name} {budget}ms: {records} rows, {result['outcome']}, {dict(statuses)}", flush=True)
    return result


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {
        "scope": "offline protection only; no hardware or walking acceptance",
        "deployment_allowed": False,
        "replay_executable_sha256": digest(EXE),
        "results": [],
    }
    for name, (relative, sha, count) in CAPTURES.items():
        for budget in (40, 1000):
            report["results"].append(run(name, relative, sha, count, budget))
            (OUT / "replay-assessment.partial.json").write_text(json.dumps(report, indent=2)+"\n")
    report["protection_acceptance"] = "PASS"
    report["continuous_operation_acceptance"] = "NOT_PASSED"
    report["source_row_count_including_overlap"] = sum(c[2] for c in CAPTURES.values())
    keys = {}
    for name, (relative, _, _) in CAPTURES.items():
        with (ROOT / relative).open() as source:
            keys[name] = {
                (r["host_read_end_ns"], tuple(r["raw"])) for r in map(json.loads, source)
            }
    assert keys["short_healthy_full"] <= keys["long_inconsistent"]
    report["overlapping_snapshot_rows"] = len(keys["short_healthy_full"])
    report["distinct_time_payload_pairs"] = len(set.union(*keys.values()))
    report["overlap_note"] = "short_healthy_full is a prefix snapshot of long_inconsistent, not an independent run"
    (OUT / "replay-assessment.json").write_text(json.dumps(report, indent=2)+"\n")
    print("PASS: offline protection. NOT PASSED: uninterrupted operation / physical balance.")


if __name__ == "__main__":
    main()
