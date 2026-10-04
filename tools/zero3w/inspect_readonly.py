#!/usr/bin/env python3
"""Passive Zero3W inventory; run ONLY through audited_script.py.

No service mutation, driver probing, bus open, device ioctl or control RPC.
Existing daemons may already own hardware; subscribing observes their data,
it does not prove their bus is safe or that eleven individual IDs responded.
No argv/environment/account dumps: configuration and identity use allowlists.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import time

try:
    import tomllib
except ImportError:
    tomllib = None

UNITS = ['robotd', 'updaterd', 'configd', 'padd', 'btd', 'tofd', 'mediad', 'rkaiq']


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def command(argv):
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=8)
        return {'argv': argv, 'exit_code': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    except Exception as error:
        return {'argv': argv, 'error': str(error)}


def file_info(path):
    p = Path(path)
    row = {'path': str(p), 'resolved': str(p.resolve()), 'exists': p.exists()}
    if p.is_file():
        row.update(bytes=p.stat().st_size, sha256=sha(p))
    return row


def json_file(path, keys):
    p = Path(path)
    if not p.is_file():
        return {'path': str(p), 'exists': False}
    try:
        obj = json.loads(p.read_text())
        return {'path': str(p), 'mtime_ns': p.stat().st_mtime_ns,
                'data': {k: obj[k] for k in keys if k in obj}}
    except Exception as error:
        return {'path': str(p), 'error': str(error)}


def observe(path, method, count=0, notification=None):
    """Bounded subscriptions to documented read routes. No implicit init/enable."""
    allowed = {'hello', 'robot.health', 'robot.subscribe', 'tof.stream', 'head_imu.stream'}
    if method not in allowed:
        raise ValueError('not a read-only route')
    result = {'socket': path, 'method': method, 'received_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(4); s.connect(path)
            params = {'api_version': 37} if method == 'hello' else {}
            s.sendall(json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}).encode() + b'\n')
            with s.makefile('rb') as stream:
                deadline = time.monotonic() + 4
                frames = []
                if count:
                    result['frames'] = frames
                while time.monotonic() < deadline:
                    line = stream.readline(2 * 1024 * 1024)
                    if not line:
                        break
                    obj = json.loads(line)
                    if obj.get('id') == 1:
                        result['reply'] = obj
                        if obj.get('error') or count == 0:
                            break
                    elif obj.get('method') == notification:
                        p = obj.get('params', {})
                        row = {'received_monotonic_ns': time.monotonic_ns(), 'data': p}
                        # t_ns shares the board CLOCK_MONOTONIC, not laptop wall time.
                        if p.get('t_ns', 0) > 0:
                            row['sample_age_ms'] = (row['received_monotonic_ns'] - p['t_ns']) / 1e6
                        frames.append(row)
                        if len(frames) >= count:
                            break
                if count:
                    result['frames'] = frames
    except Exception as error:
        result['error'] = str(error)
    return result


def main():
    result = {'kind': 'passive_inventory_not_hardware_acceptance',
              'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'machine': platform.machine(), 'kernel': platform.release(),
              'hostname': socket.gethostname(), 'uid': os.geteuid(),
              'python': platform.python_version(), 'libc': platform.libc_ver(),
              'remote_files_written': False, 'services_mutated': False,
              'device_buses_opened': False}
    result['system'] = [command(['df', '-h', '/opt', '/run', '/tmp']),
                        command(['uptime']), command(['timedatectl', 'show', '--property=NTPSynchronized,Timezone'])]
    units = {}
    config_paths = {Path('/etc/robot/robotd.toml')}
    for name in UNITS:
        status = command(['systemctl', 'show', name + '.service',
                          '--property=LoadState,ActiveState,SubState,UnitFileState,FragmentPath,DropInPaths,MainPID,NRestarts,Result'])
        fields = dict(line.split('=', 1) for line in status.get('stdout', '').splitlines() if '=' in line)
        row = {'properties': fields, 'query_exit_code': status.get('exit_code'), 'query_error': status.get('error')}
        pid = fields.get('MainPID', '0')
        if pid != '0':
            exe = Path('/proc') / pid / 'exe'
            if exe.exists():
                row['executable'] = file_info(exe)
            # Only flags relevant to effective configuration/fake distinction.
            try:
                argv = (Path('/proc') / pid / 'cmdline').read_bytes().decode().split('\0')
                row['fake_backend_flag'] = '--fake' in argv
                row['overrides'] = {flag: argv[i + 1] for i, flag in enumerate(argv[:-1])
                                    if flag in ('--params', '--socket')}
                if name == 'robotd' and '--params' in row['overrides']:
                    config_paths.add(Path(row['overrides']['--params']))
                row['open_device_paths'] = sorted({os.readlink(p) for p in (Path('/proc') / pid / 'fd').iterdir()
                                                  if p.is_symlink() and os.readlink(p).startswith('/dev/')})
            except Exception as error:
                row['process_metadata_error'] = str(error)
        paths = [fields.get('FragmentPath', ''), *fields.get('DropInPaths', '').split()]
        row['unit_files'] = [file_info(p) for p in paths if p]
        row['identity'] = json_file('/run/' + name + '/identity.json',
                                    ('service', 'version', 'revision', 'built_at', 'exe', 'pid'))
        units[name] = row
    result['units'] = units
    configs = []
    for p in sorted(config_paths):
        row = file_info(p)
        if p.is_file() and tomllib:
            try:
                obj = tomllib.loads(p.read_text())
                row['selected'] = {k: obj[k] for k in ('bus', 'policy', 'control', 'safety', 'imu', 'head_imu', 'camera', 'update_gate') if k in obj}
            except Exception as error:
                row['parse_error'] = str(error)
        elif p.is_file():
            row['parse_error'] = 'Python <3.11: no tomllib; file hashed only'
        configs.append(row)
    result['robot_configs'] = configs
    result['official_binary'] = file_info('/opt/robot/daemon/current/bin/robotd')
    result['updater_config'] = file_info('/etc/robot/updater.toml')
    # Whitelist updater keys: remote/account credentials must never reach logs.
    up = Path('/etc/robot/updater.toml')
    if up.is_file() and tomllib:
        obj = tomllib.loads(up.read_text())
        result['updater_selected'] = {k: obj[k] for k in ('channel', 'check_interval_secs', 'auto_update', 'auto_apply', 'robot_socket') if k in obj}
    roots = [Path('/opt/robot/policies'), Path('/opt/robot/policies/current').resolve(),
             Path('/opt/robot/daemon/current').resolve()]
    found = {}
    for root in roots:
        if root.is_dir():
            for p in root.rglob('*.onnx'):
                if p.is_file():
                    found.setdefault(str(p.resolve()), file_info(p))
    result['onnx_files'] = list(found.values())
    result['runtime_libraries'] = []
    for root in (Path('/usr/lib'), Path('/usr/local/lib'), Path('/opt/robot/daemon/current').resolve()):
        if root.is_dir():
            for p in root.rglob('libonnxruntime.so*'):
                if p.is_file() and not any(x['resolved'] == str(p.resolve()) for x in result['runtime_libraries']):
                    result['runtime_libraries'].append(file_info(p))
    result['devices'] = {pattern: [str(p) for p in sorted(Path('/dev').glob(pattern))]
                         for pattern in ('ttyS*', 'ttyUSB*', 'ttyACM*', 'i2c-*', 'video*', 'media*')}
    result['camera_sysfs'] = [{'node': p.name, 'name': (p / 'name').read_text().strip(), 'resolved': str(p.resolve())}
                              for p in Path('/sys/class/video4linux').glob('*') if (p / 'name').is_file()]
    result['i2c_sysfs'] = [{'node': p.name, 'name': (p / 'name').read_text().strip()}
                          for p in Path('/sys/class/i2c-dev').glob('*') if (p / 'name').is_file()]
    result['tools_present'] = {tool: shutil.which(tool) for tool in ('v4l2-ctl', 'media-ctl', 'gst-launch-1.0', 'i2cdetect', 'robotctl')}
    camera_keys = ('fps', 'targetFps', 'width', 'height', 'format', 'frames', 'dropped', 'consumers')
    result['camera_stats_before'] = json_file('/run/mediad/camera.json', camera_keys)
    robot_socket = units['robotd'].get('overrides', {}).get('--socket', '/run/robotd.sock')
    result['robot_hello'] = observe(robot_socket, 'hello')
    result['robot_health'] = observe(robot_socket, 'robot.health')
    result['robot_state'] = observe(robot_socket, 'robot.subscribe', 10, 'robot.state')
    result['tof'] = observe('/run/tofd/tof.sock', 'tof.stream', 5, 'tof.frame')
    result['head_imu'] = observe('/run/tofd/tof.sock', 'head_imu.stream', 5, 'head_imu.frame')
    time.sleep(1)
    result['camera_stats_after'] = json_file('/run/mediad/camera.json', camera_keys)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
