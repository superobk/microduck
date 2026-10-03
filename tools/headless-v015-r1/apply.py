#!/usr/bin/env python3
"""Apply/reverse the source patch against the exact official 0.15.0 commit.

No network access and no dependency beyond Python 3.10 and git. Every source
preimage is checked by Git blob hash and every replacement by occurrence count.
All changes are validated before any source is written. Never edits weights.
"""
from __future__ import annotations
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import sys

ROOT = Path(__file__).resolve().parent
# A directory walk must never promote .DS_Store or an accidental executable into
# the control repository. This is an importer for the three reviewed Rust files,
# not a general file-copy facility. The zippack originals remain untouched.
PAYLOAD_FILES = (
    "duck-control/src/morphology.rs",
    "robotd/src/headless.rs",
    "robotd/tests/headless_profile.rs",
)


def git(repo: Path, *args: str) -> bytes:
    p = subprocess.run(["git", "-C", str(repo), *args], capture_output=True)
    if p.returncode:
        raise RuntimeError(p.stderr.decode(errors="replace").strip())
    return p.stdout


def blob_hash(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def replace_checked(text: str, edit: dict) -> str:
    found = text.count(edit["old"])
    if found != edit["count"]:
        raise RuntimeError(f"{edit['name']}: expected {edit['count']} exact source matches, got {found}")
    return text.replace(edit["old"], edit["new"])


def transform(data: bytes, edits: list[dict]) -> bytes:
    text = data.decode("utf-8")
    for edit in edits:
        text = replace_checked(text, edit)
    return text.encode("utf-8")


def safe_target(repo: Path, relative: str) -> Path:
    p = repo / relative
    if Path(relative).is_absolute() or ".." in Path(relative).parts:
        raise RuntimeError(f"unsafe path {relative}")
    if p.is_symlink() or repo not in p.resolve().parents:
        raise RuntimeError(f"symlink/out-of-tree path refused: {p}")
    return p


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = path.stat().st_mode & 0o777 if path.exists() else 0o644
    fd, name = tempfile.mkstemp(prefix=".headless-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data); f.flush(); os.fsync(f.fileno())
        os.chmod(name, mode)
        os.replace(name, path)
    finally:
        if os.path.exists(name): os.unlink(name)


def payload_sources(root: Path) -> list[tuple[str, Path]]:
    """Only declared Rust sources can become patch targets, even on Finder copies."""
    unexpected = [p for p in (root / "payload").rglob("*")
                  if p.is_file() and p.name != ".DS_Store"
                  and p.relative_to(root / "payload").as_posix() not in PAYLOAD_FILES]
    if unexpected:
        raise RuntimeError(f"undeclared payload files: {unexpected}")
    sources=[]
    for relative in PAYLOAD_FILES:
        src=root / "payload" / relative
        if src.is_symlink() or not src.is_file():
            raise RuntimeError(f"missing/symlink payload refused: {relative}")
        sources.append((relative,src))
    return sources


def plan(repo: Path, manifest: dict, reverse: bool) -> list[tuple[str, bytes | None, bytes | None]]:
    base = manifest["upstream_commit"]
    head = git(repo, "rev-parse", "HEAD").decode().strip()
    # A committed descendant is okay only if the locked base is its ancestor and
    # all affected files are exactly known preimages/postimages. A random release isn't.
    git(repo, "merge-base", "--is-ancestor", base, head)
    items = []
    for relative, spec in manifest["files"].items():
        original = git(repo, "show", f"{base}:{relative}")
        if blob_hash(original) != spec["git_blob_sha1"]:
            raise RuntimeError(f"upstream blob mismatch: {relative}")
        modified = transform(original, spec["edits"])
        path = safe_target(repo, relative)
        current = path.read_bytes()
        source, target = (modified, original) if reverse else (original, modified)
        if current not in (source, target):
            raise RuntimeError(f"local changes in {relative}; use a clean dedicated worktree")
        items.append((relative, current, target))
    for relative, src in payload_sources(ROOT):
        path = safe_target(repo, relative)
        current = path.read_bytes() if path.exists() else None
        supplied = src.read_bytes()
        if current not in (None, supplied):
            raise RuntimeError(f"refusing to overwrite existing {relative}")
        items.append((relative, current, None if reverse else supplied))
    return items


def run() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", type=Path, required=True)
    ap.add_argument("--check", action="store_true", help="validate only; do not write")
    ap.add_argument("--reverse", action="store_true", help="restore guarded source files; never deploys/restarts")
    args = ap.parse_args()
    repo = args.repo.resolve()
    manifest = json.loads((ROOT / "edits.json").read_text())
    items = plan(repo, manifest, args.reverse)
    for rel, old, new in items:
        print(("UNCHANGED" if old == new else "REMOVE" if new is None else "WRITE"), rel)
    if args.check:
        print("CHECK PASSED: all exact upstream hashes and source anchors matched; no files written.")
        return 0
    completed = []
    try:
        for rel, old, new in items:
            if old == new: continue
            path = safe_target(repo, rel)
            if new is None: path.unlink(missing_ok=True)
            else: atomic_write(path, new)
            completed.append((path, old))
    except Exception:
        for path, old in reversed(completed):
            if old is None: path.unlink(missing_ok=True)
            else: atomic_write(path, old)
        raise
    diff = []
    for rel, old, new in items:
        if old == new: continue
        diff.extend(difflib.unified_diff(
            (old or b"").decode().splitlines(True), (new or b"").decode().splitlines(True),
            fromfile=f"a/{rel}" if old is not None else "/dev/null",
            tofile=f"b/{rel}" if new is not None else "/dev/null"))
    out = ROOT / ("generated-reverse.patch" if args.reverse else "generated.patch")
    atomic_write(out, "".join(diff).encode())
    print(f"Source changes applied. Review: git -C {repo} diff")
    print(f"Complete unified diff written to {out}")
    print("NOT compiled, installed or executed by this tool. Next: cargo test --locked -p duck-control -p robotd")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except (OSError, RuntimeError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise SystemExit(1)
