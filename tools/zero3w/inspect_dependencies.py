#!/usr/bin/env python3
"""Second passive check: explain missing policy/IMU/bus and runtime ambiguity.

Read files, loader metadata and already-written logs only. Never opens hardware,
restarts a service, installs a package, updates a model or changes configuration.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tomllib


def meta(p):
    p = Path(p)
    obj = {'path': str(p), 'exists': p.exists(), 'symlink': p.is_symlink(), 'resolved': str(p.resolve())}
    if p.is_file():
        obj['sha256'] = hashlib.sha256(p.read_bytes()).hexdigest()
    elif p.is_dir():
        obj['entries'] = [{'name': c.name, 'symlink': c.is_symlink(), 'resolved': str(c.resolve())}
                          for c in sorted(p.iterdir())][:100]
    return obj


def main():
    obj = {'kind': 'passive_dependency_followup', 'remote_files_written': False, 'services_mutated': False}
    obj['policy_paths'] = [meta(p) for p in ('/opt/robot/policies', '/opt/robot/policies/current',
                                            '/opt/robot/daemon/current/models', '/opt/robot/daemon/current/policies')]
    conf = tomllib.loads(Path('/etc/robot/updater.toml').read_text())
    obj['updater_selected'] = {k: conf[k] for k in ('check_interval', 'auto_apply') if k in conf}
    obj['update_components'] = {}
    for name, value in conf.get('component', {}).items():
        source = value.get('source', {})
        obj['update_components'][name] = {k: source[k] for k in ('type', 'channel', 'tag_prefix') if k in source}
    unit_paths = ('/etc/systemd/system/robotd.service', '/etc/systemd/system/updaterd.service', '/etc/systemd/system/mediad.service')
    obj['unit_execution_selected'] = {}
    for name in unit_paths:
        # Service args are known executable paths; redact any unexpected flags.
        lines = Path(name).read_text().splitlines()
        selected = [line for line in lines if line.startswith(('User=', 'Group=', 'SupplementaryGroups=', 'DeviceAllow=', 'ConditionPathExists='))]
        selected += [line for line in lines if line.startswith('ExecStart=') and all(x not in line.lower() for x in ('token', 'password', 'secret', 'credential'))]
        obj['unit_execution_selected'][name] = selected
    proc = subprocess.run(['systemctl', 'show', 'robotd.service', '--property=MainPID', '--value'], capture_output=True, text=True, timeout=5)
    pid = proc.stdout.strip()
    if pid.isdigit() and pid != '0':
        env = (Path('/proc') / pid / 'environ').read_bytes().split(b'\0')
        # Only library/profile paths, never an environment dump.
        obj['process_path_overrides'] = {k.decode(): v.decode() for item in env if b'=' in item
                                         for k, v in [item.split(b'=', 1)]
                                         if k in (b'ORT_DYLIB_PATH', b'MICRODUCK_MORPHOLOGY', b'DUCK_RUNTIME_DIR')}
        obj['loaded_ort_mappings'] = [line.split()[-1] for line in (Path('/proc') / pid / 'maps').read_text().splitlines()
                                      if 'libonnxruntime' in line]
    obj['runtime_versions'] = []
    for path in ('/usr/lib/aarch64-linux-gnu/libonnxruntime.so.1.21.0', '/usr/local/lib/libonnxruntime.so.1.28.0'):
        row = meta(path)
        if Path(path).is_file():
            # Isolate dlopen/C-version lookup; a bad library must not hide other evidence.
            code = "import ctypes; L=ctypes.CDLL(" + repr(path) + "); F=ctypes.CFUNCTYPE(ctypes.c_char_p); "
            code += "B=type('B',(ctypes.Structure,),{'_fields_':[('api',ctypes.c_void_p),('version',F)]}); L.OrtGetApiBase.restype=ctypes.POINTER(B); print(L.OrtGetApiBase().contents.version().decode())"
            p = subprocess.run(['python3', '-c', code], capture_output=True, text=True, timeout=5)
            row.update(exit_code=p.returncode, c_api_version=p.stdout.strip(), error=p.stderr.strip())
        obj['runtime_versions'].append(row)
    p = subprocess.run(['ldconfig', '-p'], capture_output=True, text=True, timeout=5)
    obj['loader_ort_entries'] = [line.strip() for line in p.stdout.splitlines() if 'libonnxruntime' in line]
    for device in ('/dev/ttyS2', '/dev/i2c-3', '/dev/i2c-pihat'):
        row = meta(device)
        if Path(device).exists():
            stat = Path(device).stat(); row.update(mode=oct(stat.st_mode & 0o777), uid=stat.st_uid, gid=stat.st_gid)
        obj.setdefault('device_metadata', []).append(row)
    # Retain already-completed queries even if a later log/device format differs.
    print(json.dumps({'checkpoint': 'dependency_inventory', 'data': obj}), flush=True)
    p = subprocess.run(['journalctl', '-u', 'robotd.service', '--no-pager', '-n', '80', '-o', 'json'], capture_output=True, text=True, timeout=8)
    messages = []
    for line in p.stdout.splitlines():
        try:
            r = json.loads(line); m = r.get('MESSAGE', '')
            # journald JSON encodes a non-UTF8 MESSAGE as a byte array. Treat it
            # as decoded diagnostic text rather than aborting the whole audit.
            if isinstance(m, list) and all(isinstance(v, int) and 0 <= v <= 255 for v in m):
                m = bytes(m).decode('utf-8', errors='replace')
            if not isinstance(m, str):
                continue
            if any(term in m.lower() for term in ('startup', 'motor', 'serial', 'imu', 'policy', 'bus')) and not any(term in m.lower() for term in ('password', 'token', 'secret', 'credential')):
                messages.append({'realtime_us': r.get('__REALTIME_TIMESTAMP'), 'message': m})
        except ValueError:
            pass
    obj['robotd_recent_diagnostics'] = {'exit_code': p.returncode, 'messages': messages[-20:]}
    obj['approval_markers'] = [meta('/etc/robot/' + n) for n in ('ZERO3W_FULL15_RESTORED', 'ZERO3W_OFFICIAL_UPDATE_REVIEWED', 'ZERO3W_HARDWARE_APPROVED')]
    obj['candidate_conflicts'] = [meta('/etc/systemd/system/' + p) for p in ('zero3w-11servo-candidate.service', 'robotd.service.d/91-zero3w-hold.conf', 'updaterd.service.d/91-zero3w-hold.conf')]
    print(json.dumps(obj, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
