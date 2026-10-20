#!/usr/bin/env python3
"""Explicit supported experiments. Never run at boot. No firmware writes."""
import argparse
import fcntl
import hashlib
import json
import math
import os
import re
from pathlib import Path
import select
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time

import capture

BASE = Path("/var/lib/robot/diagnostics/imu-acceptance")
TRACE = Path("/var/lib/robot/diagnostics/imu-acceptance-trace")
TRACE_FILE = None
RUN_DEADLINE = float("inf")
AUDIT_DIR = None


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def rpc(method, params=None):
    entry = dict(method=method, params=params or {}, **capture.stamp())
    try:
        result = _rpc(method, params)
        entry["result"] = result
        return result
    except Exception as exc:
        entry["error"] = repr(exc)
        raise
    finally:
        if AUDIT_DIR:
            with (AUDIT_DIR / "commands.jsonl").open("a") as f:
                f.write(json.dumps(entry) + "\n")


def _rpc(method, params=None):
    if method in ("robot.init", "robot.enable", "robot.move") and time.monotonic() > RUN_DEADLINE:
        raise RuntimeError("本轮超时，禁止继续动力指令")
    with socket.socket(socket.AF_UNIX) as sock:
        sock.settimeout(3)
        sock.connect("/run/robotd.sock")
        sock.sendall((json.dumps(dict(jsonrpc="2.0", id=1, method=method,
                                     params=params or {})) + "\n").encode())
        with sock.makefile() as stream:
            result = json.loads(stream.readline())
    if "error" in result:
        raise RuntimeError(str(result))
    return result.get("result", result)


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout.strip()


def relax(out):
    # An accepted RPC is not evidence of torque-off. Require a fresh control-loop log.
    cursor = run("journalctl", "-u", "robotd", "-n", "0", "--show-cursor", "--no-pager")
    cursor = cursor.split("-- cursor: ")[-1].strip()
    result = rpc("robot.relax")
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        log = run("journalctl", "-u", "robotd", "--after-cursor=" + cursor, "--no-pager")
        if "robot.relax: torque off" in log:
            confirmation = dict(result=result, cursor=cursor, **capture.stamp())
            save(out / "relax-confirmed.json", confirmation)
            return confirmation
        time.sleep(.25)
    raise RuntimeError("未收到新的 torque off 确认。保持支撑，不能判为完成。")


def identity(snapshot):
    return {k: snapshot.get(k) for k in
            ("robotd_sha256", "config_sha256", "models_sha256", "policies_path",
             "machine_id", "boot_id", "invocation_id")}


def prerequisite(stage, snapshot):
    if stage == "bench":
        return
    name = "bench" if stage == "stand" else "stand"
    path = BASE / (name + "-pass.json")
    command = "bash tools/zero3w_imu_accept.sh --stage " + name
    if name == "bench":
        command += " --long"
    try:
        prior = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"缺少有效的{name}通过记录（{path.name}）。先在Mac运行：{command}；"
                           "看到PASS后再继续。") from exc
    if prior.get("identity") != identity(snapshot) or prior.get("verdict") != "PASS":
        raise RuntimeError(f"上阶段未通过或身份改变。先重新运行：{command}")


def cancel_watchdog(name, released):
    # An unrelated experiment error must not leave a delayed reset armed.
    if released:
        run("systemctl", "stop", name + ".timer", name + ".service")


def clear_stale_watchdogs(out):
    units = run("systemctl", "list-units", "--all", "--plain", "--no-legend",
                "imu-accept-stop-*.timer", "imu-accept-stop-*.service")
    names = set()
    for line in units.splitlines():
        unit = line.split()[0] if line.split() else ""
        match = re.fullmatch(r"(imu-accept-stop-(\d+))\.(timer|service)", unit)
        if match:
            if Path("/proc", match[2]).exists():
                raise RuntimeError("另一轮验收进程仍存在；结束该轮后再试")
            names.add(match[1])
    for name in sorted(names):
        cancel_watchdog(name, True)
    save(out / "stale-watchdogs-cleared.json", sorted(names))


def journal_evidence(entries, release, start, end):
    """Journal timestamps share the board's monotonic clock with raw payload capture."""
    resets, torque_on = [], []
    release_seen = False
    for entry in entries:
        ns = int(entry["__MONOTONIC_TIMESTAMP"]) * 1000
        message = entry.get("MESSAGE", "")
        if (release["monotonic_ns"] - 10_000_000_000 <= ns <= release["monotonic_ns"] and
                "robot.relax: torque off" in message):
            release_seen = True
        if release["monotonic_ns"] <= ns <= end and "torque on=true" in message:
            torque_on.append(ns)
        if start <= ns <= end and ("robot.relax: torque off" in message or
                                  "IMU quality monitor rearmed" in message):
            resets.append(ns)
    return dict(release_confirmed=release_seen, journal_checked=release_seen,
                torque_on_ns=torque_on, reset_ns=resets)


def assess(rows, stage, states, physical_ok, complete, evidence=None):
    """Fail closed: incomplete evidence never authorizes a subsequent powered stage."""
    errors = []
    if not complete or not physical_ok:
        errors.append("操作未完整完成，或实物验收未通过")
    if len(rows) < 500 or not states:
        errors.append("有效记录不足")
    evidence = evidence or {}
    if evidence.get("journal_checked") is not True:
        errors.append("缺少本轮控制状态日志核对")
    if evidence.get("reset_ns"):
        errors.append("采集窗口收到relax/重新布防，验收被中断；不是未经relax自动恢复")
    if stage == "bench" and (evidence.get("release_confirmed") is not True or
                             evidence.get("torque_on_ns")):
        errors.append("支架阶段缺少新torque off确认，或之后出现torque on")
    unknown = 0
    latched = False
    previous = None
    for row in rows:
        d = row["decoded"]
        q = d["quality"]
        status = q["status"]
        if row.get("dropped_records") != 0 or d.get("quality_mode") != "predict":
            errors.append("丢记录或保护模式不符")
        if status == "unknown":
            latched = True
            unknown += 1
            if d.get("selected_valid") is not False or d.get("control_ready") is not False:
                errors.append("无效姿态仍可进入控制")
        elif latched and not evidence.get("reset_ns"):
            errors.append("未经 relax 自动恢复")
        if status == "predicted":
            if (q["since_good_ms"] > 40.000001 or previous is None or
                    row["host_read_end_ns"] - previous > 40_000_000):
                errors.append("预测超出40ms边界")
        previous = row["host_read_end_ns"]
        if status not in ("bootstrap", "good", "heading_aligned", "predicted", "unknown"):
            errors.append("未知质量状态")
        if status != "unknown" and d.get("selected_valid") is not True:
            errors.append("有效性字段缺失/不一致")
    for state in states:
        p = state.get("params", {})
        # safety.limp means LimpFall::Limp, not general torque-off.
        if stage != "bench":
            safety = p.get("safety", {})
            if (safety.get("limp") is not False or safety.get("fallen") is not False or
                    safety.get("picked_up") is not False or
                    p.get("policy") in (None, "held", "homing", "limp_fall", "limp_pose", "rest")):
                errors.append("动力观察段未持续正常运行策略")
        if stage in ("bench", "stand"):
            move = p.get("move", {})
            for key in ("requested", "applied"):
                v = move.get(key, [])
                if len(v) != 3 or any(not math.isfinite(x) or abs(x) > 1e-6 for x in v):
                    errors.append("零指令阶段存在运动指令或字段缺失")
        else:
            for key in ("requested", "applied"):
                v = p.get("move", {}).get(key, [])
                if (len(v) != 3 or any(not math.isfinite(x) for x in v) or
                        not -1e-6 <= v[0] <= .051 or abs(v[1]) > 1e-6 or abs(v[2]) > 1e-6):
                    errors.append("行走指令超出本轮小幅前进范围")
    if stage != "bench" and unknown:
        errors.append("动力阶段触发保护，不能进入下一阶段")
    if stage == "walk" and not any(s.get("params", {}).get("move", {}).get(
            "applied", [0])[0] > .01 for s in states):
        errors.append("未观察到前进指令实际应用")
    verdict = "FAIL" if errors else ("NOT_EXERCISED" if stage == "bench" and unknown < 50 else "PASS")
    return dict(stage=stage, verdict=verdict, errors=sorted(set(errors)), rows=len(rows),
                unknown_rows=unknown, physical_ok=physical_ok,
                scope="supported_torque_off_guard" if stage == "bench" else "supervised_short_test",
                actuator_write_observation="software tests only; trace is not a bus sniffer",
                next_stage_allowed=verdict == "PASS")


def mark(out, label):
    with (out / "events.jsonl").open("a") as f:
        f.write(json.dumps(dict(label=label, **capture.stamp())) + "\n")
    print("标签：" + label, flush=True)


def confirm(message):
    print(message + "\n30秒内请输入 YES 回车（其他输入/超时终止）：", flush=True)
    if not select.select([sys.stdin], [], [], 30)[0]:
        raise RuntimeError("确认超时")
    answer = sys.stdin.readline().strip()
    if answer != "YES":
        raise RuntimeError("用户未确认")


def latest_trace():
    paths = [TRACE_FILE] if TRACE_FILE else list(TRACE.glob("imu-[0-9]*.jsonl"))
    if not paths:
        raise RuntimeError("缺少原始IMU记录")
    path = max(paths, key=lambda p: p.stat().st_mtime_ns)
    with path.open("rb") as f:
        f.seek(max(0, path.stat().st_size - 16384))
        lines = f.read().splitlines()
    for line in reversed(lines):
        try:
            row = json.loads(line)
            if time.monotonic_ns() - row["host_read_end_ns"] > 1_000_000_000:
                raise RuntimeError("IMU记录超过1秒未更新")
            return row
        except (ValueError, KeyError):
            continue
    raise RuntimeError("没有完整IMU记录")


def require_upright(row):
    """Setup check only, not a replacement for the production disagreement detector."""
    d = row["decoded"]
    if not d.get("control_ready") or d["quality"]["status"] not in ("good", "heading_aligned"):
        raise RuntimeError("姿态未稳定就绪；不发送动力指令")
    q = d.get("selected_quat", [])
    if len(q) != 4 or any(not math.isfinite(v) for v in q) or abs(sum(v*v for v in q) - 1) > .001:
        raise RuntimeError("姿态无效；不发送动力指令")
    tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q[1]**2 + q[2]**2)))))
    if tilt > 15:
        raise RuntimeError(f"近似竖直摆放但姿态倾角{tilt:.1f}°超过15°检查范围；保持relax分析")


def phase(out, name, seconds, powered=False, vx=0.0):
    mark(out, name + "_begin")
    print(f"{name}：{seconds}秒。b 回车标记后退；s 回车表示已托稳并立即停止。", flush=True)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        row = latest_trace()
        if powered and (row["decoded"]["quality"]["status"] == "unknown" or
                        not row["decoded"].get("control_ready")):
            mark(out, "imu_fault_stop")
            raise RuntimeError("姿态保护触发/未就绪，终止本轮")
        if vx:
            rpc("robot.move", dict(vx=vx, vy=0.0, vyaw=0.0))
        if select.select([sys.stdin], [], [], .1)[0]:
            value = sys.stdin.readline().strip().lower()
            if value == "b":
                mark(out, "backward_drift")
            elif value == "s" or not value:
                mark(out, "operator_supported_stop")
                raise RuntimeError("用户停止")
    if vx:
        rpc("robot.stop")
    mark(out, name + "_end")


def session(stage, long, run_name, expected_sha):
    global TRACE_FILE, RUN_DEADLINE, AUDIT_DIR
    BASE.mkdir(parents=True, exist_ok=True)
    out = BASE / run_name
    out.mkdir(mode=0o700)
    AUDIT_DIR = out
    capture_start = time.monotonic_ns()
    save(out / "started.json", dict(stage=stage, **capture.stamp()))
    # Invalidate before even attempting setup, so a failed rerun cannot reuse an older pass.
    for name in (["bench", "stand", "walk"] if stage == "bench" else
                 ["stand", "walk"] if stage == "stand" else ["walk"]):
        (BASE / (name + "-pass.json")).unlink(missing_ok=True)
    collector = None
    watchdog = "imu-accept-stop-" + str(os.getpid())
    complete = physical_ok = False
    window_start = window_end = None
    snapshot = None
    error = None
    release = None
    evidence = None
    try:
        # Other command sources stay stopped after the experiment.
        run("systemctl", "stop", "padd", "btd", "mediad")
        rpc("robot.stop")
        relax(out)
        clear_stale_watchdogs(out)
        # A queued old service may have completed during cleanup. Establish a fresh
        # release baseline only after both its timer and service have been stopped.
        release = relax(out)
        save(out / "initial-relax-confirmed.json", release)
        snapshot = capture.snapshot(out, expected_sha)
        snapshot["machine_id"] = Path("/etc/machine-id").read_text().strip()
        snapshot["invocation_id"] = run("systemctl", "show", "robotd", "-p", "InvocationID", "--value")
        save(out / "snapshot.json", snapshot)
        # Current policy.list is authoritative; startup text may have rotated out of the journal.
        issues = [issue for issue in snapshot["policy_identity_issues"]
                  if not issue.startswith("no policy loaded message")]
        if issues:
            raise RuntimeError(str(issues))
        env = run("systemctl", "show", "robotd", "-p", "Environment", "--value")
        env = dict(item.split("=", 1) for item in shlex.split(env) if "=" in item)
        if (env.get("ROBOT_IMU_MODE") != "predict" or env.get("ROBOT_IMU_MAX_PREDICT_MS") != "40" or
                env.get("ROBOT_IMU_TOLERANCE_DEG") != "5" or
                env.get("ROBOT_IMU_TRACE_DIR") != str(TRACE)):
            raise RuntimeError("必须使用predict/40ms验收候选")
        prerequisite(stage, snapshot)
        # Independent board-side timeout, including an SSH/Mac disconnection.
        # Operator must maintain a catch/support arrangement throughout powered tests.
        robotctl = shutil.which("robotctl")
        if not robotctl:
            raise RuntimeError("robotctl unavailable")
        run("systemd-run", "--unit=" + watchdog, "--on-active=300s", "--timer-property=AccuracySec=1s",
            robotctl, "robot", "relax", "--yes", "--json")
        save(out / "watchdog.json", dict(unit=watchdog, **capture.stamp()))
        RUN_DEADLINE = time.monotonic() + 270
        TRACE_FILE = max(TRACE.glob("imu-[0-9]*.jsonl"), key=lambda p: p.stat().st_mtime_ns)
        collector = subprocess.Popen([sys.executable, str(Path(capture.__file__)),
                                      "_collect", str(out), "1000"])
        for _ in range(40):
            if (out / "state.jsonl").exists() and (out / "state.jsonl").stat().st_size:
                break
            time.sleep(.25)
        else:
            raise RuntimeError("状态采集未启动")
        # Refresh the torque-off guard before measuring; warmup must finish.
        time.sleep(2)
        if stage != "bench":
            require_upright(latest_trace())
        if stage == "bench":
            # No init, no enable, no synthetic mutation of the live sensor.
            # Bench is at most180s; the independent300s relax cannot reset its measured latch.
            window_start = time.monotonic_ns()
            phase(out, "bench_fixed_static", 180 if long else 30)
            window_end = time.monotonic_ns()
            confirm("采集结束。躯干是否一直固定、无调整，腿脚无支架/手接触，舵机未动作？")
        else:
            confirm("双脚平放地面，托稳躯干，腿脚无阻挡。接下来init只到HOME（不是seat），会使关节动作。")
            require_upright(latest_trace())
            mark(out, "init_home")
            rpc("robot.init")
            phase(out, "home_supported", 4, powered=True)
            confirm("HOME已完成且未卡住。保持托稳，准备enable平衡控制。")
            require_upright(latest_trace())
            mark(out, "enable")
            rpc("robot.enable", {"on": True, "toggle": False})
            phase(out, "enable_supported", 3, powered=True)
            confirm("双脚已经承重且能站稳。准备撤手；保护绳保持松弛、随时能托住。YES后撤手。")
            window_start = time.monotonic_ns()
            phase(out, "hands_off_zero_stand", 30, powered=True)
            if stage == "walk":
                confirm("前方至少0.5米无障碍，有人随行保护。将自动以0.05m/s前进3秒，再停止。")
                require_upright(latest_trace())
                phase(out, "walk_forward", 3, powered=True, vx=.05)
                phase(out, "walk_stopped", 5, powered=True)
            window_end = time.monotonic_ns()
            confirm("请现在托稳躯干，准备relax。全程是否无倒地、持续抖动、自行后退、扶正或支架施力？")
        physical_ok = complete = True
    except (Exception, KeyboardInterrupt) as exc:
        error = repr(exc)
        print("停止：" + error, flush=True)
    finally:
        # Fix the evidence cutoff BEFORE relax resets the latch. Stop first; potentially
        # large file reads/checksums must never delay an operator's stop request.
        window_end = window_end or time.monotonic_ns()
        released = False
        for action_name, stop_action in (("stop", lambda: rpc("robot.stop")),
                                        ("relax", lambda: relax(out))):
            try:
                stop_action()
                if action_name == "relax":
                    released = True
            except Exception as exc:
                error = str(exc)
                complete = False
                print("未能确认停止/relax。必须保持物理支撑；日志不算通过。", flush=True)
        try:
            # Do this immediately, including after a user stop or failed prerequisite.
            if (out / "watchdog.json").exists():
                cancel_watchdog(watchdog, released)
        except Exception as exc:
            complete = False
            error = "定时器清理失败：" + str(exc)
        try:
            if release and window_start:
                log = run("journalctl", "-u", "robotd", "--after-cursor=" + release["cursor"],
                          "--no-pager", "-o", "json")
                (out / "control-journal.jsonl").write_text(log + "\n")
                evidence = journal_evidence([json.loads(s) for s in log.splitlines() if s],
                                            release, window_start, window_end)
                save(out / "control-evidence.json", evidence)
        except Exception as exc:
            complete = False
            error = "控制日志核对失败：" + str(exc)
        time.sleep(.1)  # Allow the trace writer to drain records preceding the fixed cutoff.
        full_rows = []
        for source in ([TRACE_FILE] if TRACE_FILE else []):
            with source.open() as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                        if capture_start <= row["host_read_end_ns"] <= window_end:
                            full_rows.append(row)
                    except ValueError:
                        continue  # incomplete writer tail; completeness is checked below
        full_rows.sort(key=lambda r: r["host_read_end_ns"])
        with (out / "imu-payload-full.jsonl").open("w") as f:
            for row in full_rows:
                f.write(json.dumps(row) + "\n")
        sealed = [row for row in full_rows if (window_start or window_end) <= row["host_read_end_ns"]]
        with (out / "imu-payload.jsonl").open("w") as f:
            for row in sealed:
                f.write(json.dumps(row) + "\n")
        if collector:
            collector.terminate()
            try:
                collector.wait(timeout=15)
            except subprocess.TimeoutExpired:
                collector.kill()
                collector.wait()
                complete = False
        try:
            end_dir = out / "end"
            end_dir.mkdir()
            end_snapshot = capture.snapshot(end_dir, expected_sha)
            end_snapshot["machine_id"] = Path("/etc/machine-id").read_text().strip()
            end_snapshot["invocation_id"] = run("systemctl", "show", "robotd", "-p", "InvocationID", "--value")
            save(end_dir / "snapshot.json", end_snapshot)
            if identity(end_snapshot) != identity(snapshot or {}):
                complete = False
                error = "采集期间程序/配置/模型/主控身份改变"
        except Exception as exc:
            complete = False
            error = repr(exc)
        states = []
        state_path = out / "state.jsonl"
        if state_path.exists():
            for line in state_path.read_text().splitlines():
                try:
                    state = json.loads(line)
                    ns = state.get("params", {}).get("t_ns", 0)
                    if (window_start or window_end) <= ns <= window_end:
                        states.append(state)
                except ValueError:
                    complete = False
        if len(states) < 500:
            complete = False
        events = out / "events.jsonl"
        if events.exists() and any(json.loads(line)["label"] == "backward_drift"
                                   for line in events.read_text().splitlines()):
            physical_ok = False
        try:
            report = assess(sealed, stage, states, physical_ok, complete, evidence)
        except (ValueError, KeyError, TypeError) as exc:
            report = dict(stage=stage, verdict="FAIL", next_stage_allowed=False,
                          errors=["记录无法完整判读：" + str(exc)])
        # Require continuous, fresh coverage of the requested window.
        if (not sealed or not window_start or
                sealed[0]["host_read_end_ns"] - window_start > 1_000_000_000 or
                window_end - sealed[-1]["host_read_end_ns"] > 1_000_000_000 or
                any(b["host_read_end_ns"] - a["host_read_end_ns"] > 100_000_000
                    for a, b in zip(sealed, sealed[1:]))):
            report.update(verdict="FAIL", next_stage_allowed=False)
            report["errors"].append("时间窗口缺头/缺尾")
        report.update(identity=identity(snapshot or {}), error=error, control_evidence=evidence)
        if error:
            report["errors"].append(error)
        save(out / "result.json", report)
        # Re-running a prerequisite invalidates an older pass, including on failure.
        pass_file = BASE / (stage + "-pass.json")
        if pass_file.exists():
            pass_file.unlink()
        if report["verdict"] == "PASS":
            save(pass_file, report)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        save(out / "finished-acceptance.json", capture.stamp())
        sums = "".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " + str(p.relative_to(out)) + "\n"
                       for p in sorted(out.rglob("*")) if p.is_file() and p.name != "SHA256SUMS")
        (out / "SHA256SUMS").write_text(sums)
    return 0 if report["verdict"] == "PASS" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["bench", "stand", "walk"], required=True)
    parser.add_argument("--long", action="store_true")
    parser.add_argument("--run", required=True)
    parser.add_argument("--sha256", required=True)
    args = parser.parse_args()
    if not __import__("re").fullmatch(r"(bench|stand|walk)-[0-9TZ-]+", args.run):
        parser.error("invalid run name")
    os.umask(0o077)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGHUP, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    BASE.mkdir(parents=True, exist_ok=True)
    with (BASE / "session.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            sys.exit("另一轮验收仍在运行，请先结束该轮。")
        sys.exit(session(args.stage, args.long, args.run, args.sha256))
