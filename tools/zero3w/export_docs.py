#!/usr/bin/env python3
"""Export canonical tracked documentation, refuse unrelated/locally edited files.

The destination is a mirror, not a second owner of the mechanism. A prior
manifest protects local edits; stale exports are removed only after verifying
their old hashes. Initial export refuses any occupied unmanaged directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--destination',type=Path,required=True);a=p.parse_args()
    source=Path(__file__).resolve().parents[2]/'docs/zero3w-11servo';dest=a.destination.resolve()
    if dest==source or source in dest.parents:raise ValueError('destination must be a separate export directory')
    manifest=dest/'EXPORT_MANIFEST.json';old=json.loads(manifest.read_text())['files'] if manifest.exists() else {}
    if dest.exists():
        for f in dest.rglob('*'):
            if f.is_symlink():raise ValueError(f'symlink refused: {f}')
            if f.is_file() and f!=manifest:
                rel=f.relative_to(dest).as_posix()
                if rel not in old or sha(f)!=old[rel]:raise ValueError(f'unmanaged/local edit refused: {f}')
    files={f.relative_to(source).as_posix():sha(f) for f in source.rglob('*') if f.is_file()}
    dest.mkdir(parents=True,exist_ok=True)
    for rel in old.keys()-files.keys():(dest/rel).unlink()
    for rel in files:
        target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/rel,target)
    manifest.write_text(json.dumps({'canonical_source':str(source),'mirror_only':True,'files':files},ensure_ascii=False,indent=2)+'\n')
    print(f'Exported {len(files)} verified files to {dest}')
if __name__=='__main__':main()
