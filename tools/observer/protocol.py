"""Protocol-2 observation transport. The physical API deliberately has no write RPC.

The controller has 15 logical slots, but this fixture only has 11 servos. Keep ID
mapping separate from packet order so a missing leg cannot become a virtual value.
"""
from __future__ import annotations
import fcntl
import math
import os
import select
import struct
import termios
import time

SERVO_IDS = [20, 21, 22, 23, 24, 34, 10, 11, 12, 13, 14]
# Match the deployed Rust DynamixelIo::open: the custom IMU board is first,
# followed by real motors. Wire response order is independent of logical slots.
# Putting IMU last can collide with motor replies on this mixed-device chain;
# keep strict CRC/order checks instead of hiding those failures in the parser.
BUS_IDS = [200] + SERVO_IDS
SLOTS = {ident: i for i, ident in enumerate([20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14])}
HOME = [0., -.0873, -.4579, -.0049, .4530, .3491, .3491, 0., 0., 0., 0., .0873, .4579, .0049, -.4530]
HEADER = b'\xff\xff\xfd\x00'
MOUNT = [math.sqrt(.5), 0., math.sqrt(.5), 0.]

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xffff
    return crc

def encode(ident, instruction, params=b''):
    body = (bytes([instruction]) + params).replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')
    packet = HEADER + bytes([ident]) + struct.pack('<H', len(body) + 2) + body
    return packet + struct.pack('<H', crc16(packet))

def decode(packet):
    if len(packet) < 10 or packet[:4] != HEADER or int.from_bytes(packet[5:7], 'little') + 7 != len(packet):
        raise ValueError('报文长度/头错误')
    if crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
        raise ValueError('CRC错误')
    stuffed = packet[7:-2]
    body = stuffed.replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
    if body.replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd') != stuffed:
        raise ValueError('非法字节填充')
    return packet[4], body[0], body[1:]

def read_request(ident, address, length):
    # Narrow register allowlist is enforced at the transport, not just in the UI.
    allowed = {(0, 3), (64, 7), (70, 1), (144, 3), (124, 12)}
    if ident not in BUS_IDS or (address, length) not in allowed:
        raise ValueError('仅允许固定身份、扭矩、状态和遥测读取')
    if ident == 200 and (address, length) not in {(0, 3), (124, 12)}:
        raise ValueError('IMU只使用已核验的12字节契约')
    return encode(ident, 2, struct.pack('<HH', address, length))

def identity_request(ident):
    # Protocol-2 Ping returns model(2) + firmware(1). EEPROM byte 2 is model
    # information, not firmware; use the same identity contract as the old probe.
    if ident not in BUS_IDS: raise ValueError('只允许真实器件身份查询')
    return encode(ident,1)

def sync_request(fast=False,ids=None):
    ids=BUS_IDS if ids is None else ids
    if not ids or len(ids)!=len(set(ids)) or any(i not in BUS_IDS for i in ids) or (fast and ids!=BUS_IDS):
        raise ValueError('只允许已核验真实ID子集；Fast必须全链')
    if 200 in ids and ids[0]!=200:raise ValueError('IMU必须按原控制器契约位于首位')
    return encode(254, 0x8a if fast else 0x82, struct.pack('<HH', 124, 12) + bytes(ids))

def parse_fast(packet):
    # Fast Sync status has no byte stuffing; check aggregate CRC and every ID.
    if len(packet) != 8 + 16 * len(BUS_IDS) or packet[:4] != HEADER or packet[4:5] != b'\xfe' or packet[7] != 0x55:
        raise ValueError('Fast整链长度错误')
    if int.from_bytes(packet[5:7], 'little') + 7 != len(packet) or crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
        raise ValueError('Fast整链CRC/声明长度错误')
    rows = []
    for i, ident in enumerate(BUS_IDS):
        row = packet[8 + i * 16:24 + i * 16]
        if row[1] != ident:
            raise ValueError('Fast设备顺序错误')
        rows.append((ident, row[0], row[2:14]))
    return rows

def qnorm(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('非有限四元数')
    n = math.sqrt(sum(v * v for v in q))
    if n < .5:
        raise ValueError('四元数长度无效')
    return [v / n for v in q]

def qmul(a, b):
    w, x, y, z = a; v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]

def qinv(q):
    return [q[0], -q[1], -q[2], -q[3]]

def rotate(q, v):
    return qmul(qmul(q, [0., *v]), qinv(q))[1:]

def euler(q):
    w, x, y, z = qnorm(q)
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]

def imu_decode(raw, mount=MOUNT):
    if len(raw) != 12:
        raise ValueError('IMU块长度错误')
    gyro_raw = list(struct.unpack_from('<hhh', raw))
    xyz = list(struct.unpack_from('<eee', raw, 6))
    n = sum(v*v for v in xyz)
    # All-zero SFLP bytes mean uninitialised in the baseline. Never display identity
    # as a measured upright pose, even though mathematical identity is valid.
    if raw[6:] == bytes(6) or not all(math.isfinite(v) for v in xyz) or n > 1.02:
        raise ValueError('SFLP未就绪或四元数无效')
    sensor = qnorm([math.sqrt(max(0., 1-n)), *xyz])
    trunk = qnorm(qmul(sensor, qinv(mount)))
    gyro = [v*.0175*math.pi/180 for v in gyro_raw]
    return {'sensor_quat': sensor, 'trunk_quat': trunk, 'gyro_sensor': gyro,
            'gyro': rotate(mount, gyro), 'gravity': rotate(qinv(trunk), [0., 0., -1.]),
            'euler': euler(trunk), 'raw_hex': raw.hex(), 'freshness': '总线应答新鲜；融合样本计数未提供'}

def servo_decode(ident, raw, status):
    if ident not in SERVO_IDS or len(raw) != 12 or status & 0x7f:
        raise ValueError('舵机数据/状态错误')
    _, current, velocity, position = struct.unpack('<hhii', raw)
    # Match duck-control/bus.rs: XL330 current unit is 1mA, velocities are
    # 0.229rev/min, positions include the servo's homing offset already.
    return {'id': ident, 'slot': SLOTS[ident], 'position': position*2*math.pi/4096-math.pi,
            'velocity': velocity*.229*2*math.pi/60, 'current_ma': abs(current),
            'signed_current_ma': current, 'status_byte': status, 'raw_hex': raw.hex()}

class ReadOnlyPort:
    """Linux UART with cooperative flock, kernel exclusivity and restored termios."""
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.old = None
        self.exclusive=False
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.exclusive=True
            self.old = termios.tcgetattr(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B1000000
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except BaseException:
            self.close(); raise

    def close(self):
        if self.fd is not None:
            try:
                if self.old is not None: termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
                if self.exclusive:fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd); self.fd = None

    def _exchange(self, request, ids, fast=False, timeout=.025):
        # Only requests constructed by the closed allowlist above reach this method.
        termios.tcflush(self.fd, termios.TCIFLUSH)
        if os.write(self.fd, request) != len(request): raise OSError('UART短写')
        buf = bytearray(); rows = []; deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], max(0., deadline-time.monotonic()))
            if not ready: break
            buf.extend(os.read(self.fd, 4096))
            while True:
                start = buf.find(HEADER)
                if start < 0:
                    if len(buf) > 3: del buf[:-3]
                    break
                del buf[:start]
                if len(buf) < 7: break
                size = int.from_bytes(buf[5:7], 'little') + 7
                if not 10 <= size <= 1024: raise ValueError('响应长度无效')
                if len(buf) < size: break
                packet = bytes(buf[:size]); del buf[:size]
                if packet[7] != 0x55: continue  # adapter echo is not telemetry
                if fast: return parse_fast(packet)
                ident, kind, data = decode(packet)
                if kind != 0x55 or not data or ident != ids[len(rows)]:
                    raise ValueError(f'响应ID/顺序错误: 已收{[r[0] for r in rows]}，预期{ids[len(rows)]}，实际{ident}')
                rows.append((ident, data[0], data[1:]))
                if len(rows) == len(ids): return rows
        raise TimeoutError('总线应答不完整: ' + str([r[0] for r in rows]))

    def read(self, ident, address, length):
        _, status, data = self._exchange(read_request(ident, address, length), [ident], timeout=.05)[0]
        if status & 0x7f or len(data) != length: raise ValueError('读取状态/长度错误')
        return data, status

    def identity(self,ident):
        _,status,data=self._exchange(identity_request(ident),[ident],timeout=.1)[0]
        if status & 0x7f or len(data)!=3:raise ValueError('Ping身份回包无效')
        return data,status

    def sync(self, fast=False, ids=None):
        ids=BUS_IDS if ids is None else ids
        return self._exchange(sync_request(fast,ids), ids, fast, .2 if fast else .025)
