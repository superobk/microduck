#!/usr/bin/env python3
"""Offline summary of robotctl monitor --json logs; NEVER connects to or moves a robot.
Python 3.11+, standard library only. Input may be bare RobotState or robot.state
JSON-RPC notifications. This is a descriptive report, NOT a safety certificate.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

LEG_SLOTS = (0, 1, 2, 3, 4, 10, 11, 12, 13, 14)
LEG_NAMES = tuple(f'{side}_{name}' for side in ('left', 'right')
                  for name in ('hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle'))


def number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value: Any, size: int) -> bool:
    return isinstance(value, list) and len(value) == size and all(number(x) for x in value)


def stats(values: list[float]) -> dict[str, float | int] | None:
    if not values:
        return None
    s = sorted(values)
    position = .95 * (len(s) - 1)
    lo, hi = math.floor(position), math.ceil(position)
    p95 = s[lo] + (s[hi] - s[lo]) * (position - lo)
    return {'n': len(s), 'min': s[0], 'mean': statistics.fmean(s),
            'p95': p95, 'max': s[-1]}


def extract(obj: Any) -> dict[str, Any] | None:
    if not isinstance(obj, dict):
        return None
    if obj.get('method') == 'robot.state':
        obj = obj.get('params')
    elif isinstance(obj.get('state'), dict):
        obj = obj['state']
    if isinstance(obj, dict) and isinstance(obj.get('safety'), dict) and 'joints' in obj:
        return obj
    return None


def summarize(lines: list[str], skip_seconds: float = 5.0) -> dict[str, Any]:
    if not number(skip_seconds) or skip_seconds < 0:
        raise ValueError('skip_seconds must be finite and non-negative')
    frames: list[dict[str, Any]] = []
    bad_json = ignored = invalid_t = 0
    for line in lines:
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
        except (ValueError, TypeError):
            bad_json += 1
            continue
        frame = extract(obj)
        if frame is None:
            ignored += 1
        elif not number(frame.get('t')):
            invalid_t += 1
        else:
            frames.append(frame)
    if not frames:
        raise ValueError('no RobotState frames with a finite t; check native monitor --json output')
    times = [float(f['t']) for f in frames]
    backwards = sum(b < a for a, b in zip(times, times[1:]))
    if backwards:
        raise ValueError('daemon t went backwards: split log by daemon restart before evaluating')
    selected = [f for f in frames if f['t'] - times[0] >= skip_seconds]
    if not selected:
        raise ValueError('no frames remain after skip_seconds')
    selected_times = [float(f['t']) for f in selected]
    positive_dt = [b-a for a,b in zip(selected_times, selected_times[1:]) if b > a]
    duplicate_times = sum(b == a for a,b in zip(selected_times, selected_times[1:]))
    gravity_bad = joint_bad = current_absent = 0
    tilt, hz = [], []
    errors: list[list[float]] = [[] for _ in LEG_SLOTS]
    currents: list[list[float]] = [[] for _ in LEG_SLOTS]
    fallen_flags: list[bool] = []
    missed: list[float] = []
    labels: dict[str, int] = {}
    limited: dict[str, int] = {}
    known_commands = nonzero_commands = 0
    for f in selected:
        safety = f['safety']
        g = safety.get('gravity')
        if vector(g, 3) and .8 <= math.sqrt(sum(x*x for x in g)) <= 1.2:
            norm = math.sqrt(sum(x*x for x in g))
            tilt.append(math.degrees(math.acos(max(-1., min(1., -g[2]/norm)))))
        else:
            gravity_bad += 1
        if isinstance(safety.get('fallen'), bool):
            fallen_flags.append(safety['fallen'])
        loop = f.get('control_loop')
        if isinstance(loop, dict):
            if number(loop.get('hz')) and loop['hz'] > 0:
                hz.append(float(loop['hz']))
            if number(loop.get('missed')):
                missed.append(float(loop['missed']))
        q, target = f.get('joints'), f.get('targets')
        if vector(q,15) and vector(target,15):
            for out, j in zip(errors, LEG_SLOTS):
                out.append(abs(q[j]-target[j]))
        else:
            joint_bad += 1
        c = f.get('currents_ma')
        if vector(c,15):
            for out,j in zip(currents, LEG_SLOTS):
                out.append(abs(float(c[j])))
        else:
            current_absent += 1
        label = f.get('policy')
        if isinstance(label,str):
            labels[label] = labels.get(label,0)+1
        movement = f.get('movement')
        if isinstance(movement,dict):
            cmd = movement.get('applied')
            if vector(cmd,3):
                known_commands += 1
                nonzero_commands += int(any(abs(v) > 1e-5 for v in cmd))
            reasons = movement.get('limited_by')
            if isinstance(reasons,list):
                for reason in reasons:
                    if isinstance(reason,str): limited[reason] = limited.get(reason,0)+1
    warnings = []
    if bad_json or invalid_t: warnings.append('malformed or invalid-t records were excluded; inspect raw log')
    if gravity_bad or joint_bad: warnings.append('some gravity/joint frames are invalid or missing')
    if not hz: warnings.append('no positive daemon-reported loop rate; subscriber cadence is NOT loop rate')
    if not fallen_flags: warnings.append('fallen flags unavailable; absence is not zero falls')
    if current_absent: warnings.append('some/all current telemetry absent; absence is not zero current')
    if any(all(v == 0 for v in c) for c in currents if c): warnings.append('all-zero current stream exists; verify real telemetry, not fake/unavailable backend')
    if selected_times[-1]-selected_times[0] < 30: warnings.append('short analyzed segment; no sustained-balance conclusion')
    if duplicate_times: warnings.append('duplicate publication timestamps; investigate/log separately')
    if any(b < a for a,b in zip(missed,missed[1:])): warnings.append('missed counter decreased; no reliable counter delta')
    return {
        'report_kind': 'descriptive_only_not_a_pass_certificate',
        'input': {'state_frames': len(frames), 'analyzed_frames': len(selected),
                  'skip_seconds': skip_seconds, 'malformed_lines': bad_json,
                  'ignored_nonstate_lines': ignored, 'invalid_t_frames': invalid_t},
        'analyzed_duration_s': selected_times[-1]-selected_times[0],
        'publication_interval_s': stats(positive_dt),
        'publication_hz_approx': (1/statistics.fmean(positive_dt)) if positive_dt else None,
        'daemon_control_loop_hz': stats(hz),
        'loop_missed_counter_delta': (missed[-1]-missed[0]) if missed and all(b>=a for a,b in zip(missed,missed[1:])) else None,
        'tilt_from_gravity_deg': stats(tilt),
        'invalid_gravity_frames': gravity_bad,
        'fallen_true_frames': sum(fallen_flags) if fallen_flags else None,
        'fallen_observed_rising_edges': sum(b and not a for a,b in zip(fallen_flags,fallen_flags[1:])) if fallen_flags else None,
        'first_frame_already_fallen': fallen_flags[0] if fallen_flags else None,
        'policy_labels_frames': labels,
        'command_nonzero_frames': nonzero_commands if known_commands else None,
        'limiter_labels_frames': limited,
        'invalid_joint_frames': joint_bad,
        'leg_target_minus_measured_abs_rad': {name:stats(v) for name,v in zip(LEG_NAMES,errors)},
        'leg_reported_current_abs_ma': {name:stats(v) for name,v in zip(LEG_NAMES,currents)},
        'warnings': warnings,
        'interpretation': [
            'movement.applied is a command, NOT measured body speed; use an external ruler/video for tracking.',
            'Published targets may precede safety clipping; same-frame target-position delta is diagnostic, not an isolated actuator-error test.',
            'Tilt-only logs cannot prove unsupported standing, no foot slip, trustworthy IMU freshness, or hardware safety.',
            'No overall PASS/FAIL is inferred. Review native health, morphology status, video and physical support conditions together.'
        ]
    }


def main() -> int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('log',type=Path)
    p.add_argument('--skip-seconds',type=float,default=5.)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    with a.log.open(encoding='utf-8') as f: report=summarize(f.readlines(),a.skip_seconds)
    text=json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if a.output:
        with a.output.open('x',encoding='utf-8') as f:f.write(text)
    print(text,end='')
    return 0

if __name__=='__main__':
    try: raise SystemExit(main())
    except (OSError,ValueError,TypeError,KeyError) as e:
        print(f'ERROR: {e}',file=sys.stderr);raise SystemExit(2)
