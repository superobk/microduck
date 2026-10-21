#!/usr/bin/env python3
"""Read-only capture worker extracted from the existing Zero3W diagnostic kit. No start/motion RPCs."""
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time

REQUIRED = ('walk', 'sitstand', 'ground_pick', 'kick_left', 'kick_right', 'roulade')
SERVICES = ('configd', 'robotd', 'bluetooth', 'btd', 'padd')
OBSERVED_SERVICES = (*SERVICES, 'mediad')
MAX_BYTES = 1024**3
MIN_FREE_BYTES = 256*1024**2


def stamp():
    return dict(wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())


def command(argv, timeout=20):
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return dict(argv=argv, exit=p.returncode, stdout=p.stdout, stderr=p.stderr)
    except (OSError, subprocess.TimeoutExpired) as e:
        return dict(argv=argv, exit=-1, error=str(e))


def decoded(result):
    if result['exit'] != 0:
        return {}
    try:
        return json.loads(result['stdout'])
    except (ValueError, KeyError):
        return {}


def policy_problems(data):
    p = data.get('policies', {})
    slots = {x['slot']: x for x in p.get('slots', [])}
    issues = []
    if not p.get('enabled'):
        issues.append('policy configuration disabled or unreadable')
    if p.get('mode') != 'walk':
        issues.append('mode is not walk')
    for name in REQUIRED:
        slot = slots.get(name, {})
        if not slot.get('path') or slot.get('error'):
            issues.append('missing/failed slot: ' + name)
    return issues


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def transport_snapshot():
    """Read-only transport identity; never opens the actuator serial device."""
    data = dict(stamp(), kernel_release=os.uname().release)
    for key, name in (
        ('boot_id', '/proc/sys/kernel/random/boot_id'),
        ('dma_names', '/sys/devices/platform/fe660000.serial/of_node/dma-names'),
        ('dmaengine', '/sys/kernel/debug/dmaengine/summary'),
        ('uart_counters', '/proc/tty/driver/serial'),
    ):
        try:
            with Path(name).open('rb') as stream:
                value = stream.read(65537)
            data[key] = dict(path=name, text=value[:65536].decode(errors='replace').replace('\0', '\n'),
                             truncated=len(value) > 65536)
        except OSError as e:
            data[key] = dict(path=name, error=str(e))
    data['kernel_uart_log'] = command([
        'journalctl', '-b', '-k', '--no-pager', '-n', '80',
        '--grep=fe660000|dma-names|got rx|use interrupt mode'], timeout=3)
    return data


def snapshot(out, expected_sha256):
    audit = dict(stamp(), boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    audit['transport'] = transport_snapshot()
    audit['services'] = command(['systemctl', 'show', *OBSERVED_SERVICES, '-p', 'Id', '-p', 'ActiveState',
                                 '-p', 'SubState', '-p', 'MainPID', '-p', 'InvocationID',
                                 '-p', 'ConditionResult', '-p', 'UnitFileState'])
    audit['robotd_exec'] = command(['systemctl', 'show', 'robotd', '-p', 'ExecStart'])
    pid_result = command(['systemctl', 'show', 'robotd', '-p', 'MainPID', '--value'])
    try:
        pid = int(pid_result['stdout'].strip())
        exe = Path('/proc') / str(pid) / 'exe'
        audit['robotd_exe'] = str(exe.resolve(strict=True))
        audit['robotd_sha256'] = hashlib.sha256(exe.read_bytes()).hexdigest()
        audit['robotd_cmdline'] = (exe.parent / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
    except (OSError, ValueError, KeyError) as e:
        audit['identity_error'] = str(e)
    for key, args in dict(policy=['policy', 'list'], pad_status=['pad', 'status'],
                          bindings=['pad', 'bindings'], version=['version']).items():
        audit[key] = command(['robotctl', *args, '--json'])
    invocation = command(['systemctl', 'show', 'robotd', '-p', 'InvocationID', '--value'])
    inv = invocation.get('stdout', '').strip()
    audit['loaded_journal'] = command(['journalctl', '_SYSTEMD_INVOCATION_ID=' + inv,
                                       '--no-pager', '-o', 'short-monotonic', '-n', '300']) if inv else {}
    audit['input_devices'] = Path('/proc/bus/input/devices').read_text()
    audit['udev'] = [command(['udevadm', 'info', '--query=property', '--name=' + str(p)])
                     for p in Path('/dev/input').glob('event*')]
    audit['pad_dropin_present'] = Path('/etc/systemd/system/padd.service.d/91-xbox-0b22-0523-mapping.conf').exists()
    audit['xbox_udev_rule_present'] = Path('/etc/udev/rules.d/99-microduck-xbox-0b22-joystick.rules').exists()
    config = Path('/etc/robot/robotd.toml')
    audit['config_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest() if config.exists() else None
    policies = Path('/opt/robot/policies/current')
    audit['policies_path'] = str(policies.resolve())
    audit['models_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in policies.glob('*.onnx')}
    issues = policy_problems(decoded(audit['policy']))
    audit['expected_robotd_sha256']=expected_sha256
    if audit.get('robotd_sha256') != expected_sha256:
        issues.append('running binary differs from expected candidate')
    if '--no-policy' in audit.get('robotd_cmdline', ''):
        issues.append('running with --no-policy')
    if 'policy loaded' not in audit.get('loaded_journal', {}).get('stdout', ''):
        issues.append('no policy loaded message in current invocation journal (may have rotated)')
    audit['policy_identity_issues'] = issues
    save(out / 'snapshot.json', audit)
    return audit


def health():
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(1)
        s.connect('/run/robotd.sock')
        s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.health"}\n')
        with s.makefile('r') as r:
            return json.loads(r.readline())


def capture(out, seconds):
    save(out / 'capture-window.json', dict(stamp(), duration_s=seconds,
                                         deadline_monotonic_ns=time.monotonic_ns()+seconds*1_000_000_000))
    stopped = threading.Event()
    stop_reason = 'duration_elapsed'
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())
    children, files = [], []
    try:
        for name, argv in (
            ('state.jsonl', ['robotctl', 'monitor', '--hz', '0', '--json']),
            ('journal.log', ['journalctl', '-b', *[a for u in OBSERVED_SERVICES for a in ('-u', u)],
                             '-n', '200', '-f', '--no-pager', '-o', 'short-monotonic'])):
            stdout, stderr = (out / name).open('wb'), (out / (name + '.stderr')).open('wb')
            files.extend([stdout, stderr])
            children.append(subprocess.Popen(argv, stdout=stdout, stderr=stderr))
        deadline = time.monotonic() + seconds
        with (out / 'health.jsonl').open('a', buffering=1) as log:
            while not stopped.is_set() and time.monotonic() < deadline:
                if shutil.disk_usage(out).free < MIN_FREE_BYTES:
                    stop_reason = 'disk_space_low'
                    break
                if sum(p.stat().st_size for p in out.iterdir() if p.is_file()) >= MAX_BYTES:
                    stop_reason = 'capture_size_limit'
                    break
                row = stamp()
                try:
                    row['response'] = health()
                except (OSError, ValueError) as e:
                    row['error'] = str(e)
                row['children'] = [p.poll() for p in children]
                log.write(json.dumps(row) + '\n')
                save(out / 'heartbeat.json', row)
                if children[0].poll() is not None:
                    stop_reason = 'state_monitor_exited'
                    break
                stopped.wait(min(2, max(0, deadline - time.monotonic())))
            if stopped.is_set():
                stop_reason = 'requested_stop'
    finally:
        stopped.set()
        for p in children:
            if p.poll() is None:
                p.terminate()
        for p in children:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for f in files:
            f.close()
        save(out / 'transport-end.json', transport_snapshot())
        save(out / 'finished.json', dict(stamp(), children=[p.returncode for p in children],
                                       torque_unchanged=True, stop_reason=stop_reason))


if __name__ == '__main__':
    os.umask(0o077)
    if len(sys.argv) != 4 or sys.argv[1] != '_collect':
        sys.exit('Internal worker: use the supported acceptance entry point.')
    capture(Path(sys.argv[2]), int(sys.argv[3]))
