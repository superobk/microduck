#!/usr/bin/env bash
# No deployment, systemd operations, ONNX replacement or hardware access.
set -euo pipefail
KIT=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 /absolute/path/to/patched/microduck [--pty]" >&2; exit 2
fi
REPO=$(cd -- "$1" && pwd)
if [[ $# == 2 && $2 != --pty ]]; then echo 'Unknown option' >&2;exit 2;fi
command -v cargo >/dev/null || { echo 'Install Rust >=1.89 first; no build has run.' >&2;exit 1; }
command -v rustc >/dev/null || exit 1
python3 -m unittest discover -s "$KIT/tests" -v
cd -- "$REPO"
git diff --check
cargo test --locked -p duck-control -p robotd
cargo build --locked -p robotd --bin robotd
if [[ ${2:-} == --pty ]]; then
  python3 "$KIT/tools/pty_bus_test.py" --robotd "$REPO/target/debug/robotd" --out "$KIT/local-test-results/pty"
fi
printf '\nNative test/build completed. This does NOT certify ONNX/physics/hardware or build an ARM cross-target.\n'
