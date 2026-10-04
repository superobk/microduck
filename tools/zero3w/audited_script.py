#!/usr/bin/env python3
"""Archive an exact reviewed Python script and execute it through audited SSH.

No host-key bypass, password arguments, or remote copy is needed for passive
checks. Python runs from an encoded in-memory loader, and every remote output
and failure is retained by the common audit wrapper. Review the script first.
"""
import argparse
import base64
import hashlib
from pathlib import Path
import shlex
import shutil
from audit_command import run


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', required=True)
    p.add_argument('--script', type=Path, required=True)
    p.add_argument('--audit-root', type=Path, required=True)
    p.add_argument('--meaning', required=True)
    p.add_argument('--note', required=True)
    a = p.parse_args()
    if a.target.startswith('-') or any(c.isspace() for c in a.target):
        p.error('invalid SSH target')
    data = a.script.read_bytes()
    archive = a.audit_root.parent / 'scripts' / hashlib.sha256(data).hexdigest()
    archive.mkdir(parents=True, exist_ok=True)
    shutil.copy2(a.script, archive / a.script.name)
    loader = 'import base64;exec(compile(base64.b64decode(' + repr(base64.b64encode(data).decode()) + "),'audited-script','exec'))"
    remote = shlex.join(['python3', '-c', loader])
    return run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=2', a.target, remote],
               a.audit_root, a.meaning + f'；归档脚本={archive / a.script.name}', a.note)


if __name__ == '__main__':
    raise SystemExit(main())
