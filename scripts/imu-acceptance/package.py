#!/usr/bin/env python3
"""Prepare the SHA-pinned supported-bench package locally. Never connects to a board."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

SHA = "d19c9ccf586c27a0b1581cab35fcad57fb9c7746e014953fb38429201d919a39"
HERE = Path(__file__).resolve().parent


def build(source, output):
    manifest = json.loads((source / "manifest.json").read_text())
    binary = source / "robotd-linux-aarch64"
    assert manifest["binary_sha256"] == SHA
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == SHA
    repo = HERE.parents[1]
    # Test-only maintenance cannot alter this deployed binary. Preserve the build's
    # original manifest hashes, and record current test hashes separately.
    test_changes = {}
    for name, expected in manifest["source_files_sha256"].items():
        actual = hashlib.sha256((repo / name).read_bytes()).hexdigest()
        if actual != expected and name.startswith(("duck-control/tests/", "robotd/tests/")):
            test_changes[name] = dict(build_sha256=expected, current_sha256=actual)
        else:
            assert actual == expected, name
    # Check every production Rust source, Cargo.lock and crate manifests, not only
    # the originally changed files listed in the audit manifest.
    files = sorted((repo / "duck-control/src").rglob("*.rs")) + sorted((repo / "robotd/src").rglob("*.rs"))
    files += [repo / "Cargo.lock", repo / "duck-control/Cargo.toml", repo / "robotd/Cargo.toml"]
    source_hash = hashlib.sha256(b"".join(
        str(p.relative_to(repo)).encode() + b"\0" + p.read_bytes() for p in files)).hexdigest()
    assert source_hash == manifest["source_hash"], "production source differs from audited binary"
    output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(binary, output / binary.name)
    for name in ("accept.py", "capture.py", "install-board.sh", "rollback-board.sh",
                 "99-imu-acceptance.conf"):
        shutil.copy2(HERE / name, output / name)
    shutil.copy2(repo / "docs/robot/imu-acceptance.md", output / "README.zh-CN.md")
    manifest.update(release_gate="SUPPORTED_BENCH_EXPERIMENT_ONLY",
                    packaging_test_only_changes=test_changes,
                    deployment_allowed=False, supported_bench_install_allowed=True,
                    physical_validation="NOT_PERFORMED",
                    note="Explicit bench invocation permits supported torque-off validation only. "
                         "Powered stages require preceding same-identity acceptance.")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output / "SHA256SUMS").write_text("".join(
        hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n"
        for p in sorted(output.iterdir()) if p.is_file() and p.name != "SHA256SUMS"))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    build(args.source, args.output)
    print(args.output)
