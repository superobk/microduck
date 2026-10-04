#!/usr/bin/env python3
"""Run an argv command with durable before/after records and complete output.

The same wrapper is used for local checks and EVERY Zero3W SSH operation. The
record is written before starting the child, so a crash leaves an identifiable
incomplete operation rather than an invented successful result. Never pass
passwords, tokens, environment dumps or other secrets as arguments or output.
"""
from __future__ import annotations
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def run(argv, root: Path, meaning: str, note: str, cwd: str | None = None):
    root.mkdir(parents=True, exist_ok=True)
    directory = root / (dt.datetime.now(dt.timezone.utc).strftime('%H%M%S') + '-' + uuid.uuid4().hex[:8])
    directory.mkdir()
    record = {'started_utc': now(), 'argv': argv, 'cwd': cwd or os.getcwd(),
              'meaning': meaning, 'next_attention': note, 'status': 'running',
              'remote': any(Path(a).name in ('ssh', 'scp') for a in argv[:2])}
    path = directory / 'operation.json'
    path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    (directory / 'README.md').write_text(f'# Operation\n\n{meaning}\n\nNext attention: {note}\n')
    print(f'AUDIT_DIR={directory}', flush=True)
    code = 127
    try:
        with (directory / 'output.log').open('wb') as out:
            process = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            try:
                while chunk := process.stdout.read1(65536):
                    out.write(chunk); out.flush()
                    sys.stdout.buffer.write(chunk); sys.stdout.buffer.flush()
                code = process.wait()
            except BaseException:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
                raise
            finally:
                process.stdout.close()
    except OSError as exc:
        (directory / 'output.log').write_text(str(exc) + '\n')
        print(exc, file=sys.stderr)
    finally:
        record.update(finished_utc=now(), exit_code=code,
                      status='success' if code == 0 else 'failed_or_interrupted')
        output = directory / 'output.log'
        if output.exists(): record['output_sha256'] = hashlib.sha256(output.read_bytes()).hexdigest()
        path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    return code


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', required=True, type=Path)
    p.add_argument('--meaning', required=True)
    p.add_argument('--note', required=True)
    p.add_argument('--cwd')
    p.add_argument('command', nargs=argparse.REMAINDER)
    a = p.parse_args()
    argv = a.command[1:] if a.command[:1] == ['--'] else a.command
    if not argv: p.error('a command is required after --')
    return run(argv, a.root, a.meaning, a.note, a.cwd)


if __name__ == '__main__':
    raise SystemExit(main())
