"""Compare *raw* fused rotation with gyro; never repair or replace measurements.

The 124/12 ABI exposes no fusion timestamp or sensor reset. A CRC-valid, fresh
reply can therefore contain a bad fused attitude. Report this separately from
transport freshness and calibration, using at most two seconds of history.
"""
import math
from protocol import qnorm


def rotation_consistency(samples, now, window=2.):
    result = {'status': '未验证', 'scope': '原始融合姿态与原始角速度的短窗一致性，非绝对精度验收'}
    rows = [(t, r) for t, r in samples if now-window <= t <= now]
    if len(rows) < 8 or now-rows[-1][0] > 1 or rows[-1][0]-rows[0][0] < 1:
        return result
    # Compare in one frame only; mount/reference changes cannot create or hide
    # this diagnostic. Sensor coordinates are preferred; robotd-only telemetry
    # falls back to its unchanged trunk coordinates for both signals.
    sensor = all('sensor_quat' in r and 'gyro_sensor' in r for _, r in rows)
    qkey, gkey = ('sensor_quat', 'gyro_sensor') if sensor else ('trunk_quat', 'gyro')
    try:
        first, last = qnorm(rows[0][1][qkey]), qnorm(rows[-1][1][qkey])
        gyros = [r[gkey] for _, r in rows]
        if any(len(g) != 3 or not all(math.isfinite(v) for v in g) for g in gyros):
            return result
        peak = max(math.degrees(math.hypot(*g)) for g in gyros)
    except (KeyError, TypeError, ValueError):
        return result
    span = rows[-1][0]-rows[0][0]
    # abs(dot) makes q and -q identical. Endpoint displacement is a lower
    # bound, so fast rotations returning to their start can escape this check.
    angle = math.degrees(2*math.acos(min(1., abs(sum(a*b for a, b in zip(first, last))))))
    rate = angle/span
    inconsistent = angle >= 3 and rate > peak+1
    result.update(status='不一致' if inconsistent else '未见明显不一致',
                  span_s=span, sample_count=len(rows), rotation_deg=angle,
                  rotation_deg_s=rate, gyro_peak_deg_s=peak,
                  reason='融合姿态转动快于原始角速度；先核对静止状态及IMU固件/初始化' if inconsistent else
                         '短窗未触发阈值，不代表姿态精度或动态方向已验收')
    return result
