#!/usr/bin/env python3
"""Passive board inventory: archive and invoke ONLY through audited_script.py.

No UART, mixer, media capture, robot RPC or service mutation occurs here. Effective
unit dependencies matter: mediad's Wants can otherwise start a held controller.
"""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import tomllib

UNITS = ['robotd', 'zero3w-11servo-candidate', 'updaterd', 'tofd', 'mediad', 'duck-observer', 'duck-observer-helper']

def command(args):
    r = subprocess.run(args, capture_output=True, text=True, timeout=15)
    return {'argv': args, 'exit_code': r.returncode, 'output': r.stdout.strip(), 'stderr': r.stderr.strip()}

def fingerprint(path):
    p = Path(path)
    if not p.is_file():
        return {'path': str(p), 'exists': False}
    return {'path': str(p), 'resolved': str(p.resolve()), 'size': p.stat().st_size,
            'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}

config = Path('/etc/robot/robotd.toml')
params = tomllib.loads(config.read_text()) if config.exists() else {}
units = {}
for name in UNITS:
    units[name] = command(['systemctl', 'show', name + '.service', '--no-pager',
        '--property=LoadState,ActiveState,SubState,MainPID,UnitFileState,FragmentPath,DropInPaths,Wants,Requires,After,Conditions,ExecStart'])
print(json.dumps({
    'uid': os.geteuid(), 'machine': platform.machine(), 'hostname': platform.node(),
    'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    'units': units, 'config': fingerprint(config),
    'loaded_robot_conditions': command(['busctl','get-property','org.freedesktop.systemd1',
        '/org/freedesktop/systemd1/unit/robotd_2eservice','org.freedesktop.systemd1.Unit','Conditions']),
    'disk': command(['df','-h','/opt/robot/local','/var/lib/robot']),
    'observation_config': {k: params.get(k, {}) for k in ['bus', 'imu', 'audio', 'media']},
    'markers': {p: Path('/etc/robot', p).exists() for p in
        ['ZERO3W_FULL15_RESTORED', 'ZERO3W_HARDWARE_APPROVED', 'ZERO3W_OFFICIAL_UPDATE_REVIEWED']},
    'alsa_cards': Path('/proc/asound/cards').read_text() if Path('/proc/asound/cards').exists() else None,
    'audio_devices': command(['aplay', '-l']),
    'audio_bank_files': [fingerprint(p) for p in sorted(Path(params.get('audio', {}).get('bank', '/var/lib/robot/sounds')).glob('chirp*.wav'))],
    'camera_stats': Path('/run/mediad/camera.json').read_text() if Path('/run/mediad/camera.json').exists() else None,
    'uart': command(['fuser', '/dev/ttyS2']),
    'ports': command(['ss', '-ltnp']),
}, ensure_ascii=False, indent=2))
