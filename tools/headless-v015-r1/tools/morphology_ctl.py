#!/usr/bin/env python3
"""Local diagnostic endpoint for patched robotd. Does not enable motors."""
from __future__ import annotations
import argparse
import json
import socket
import sys


def rpc(path: str, method: str, params: dict | None = None) -> dict:
    request = {"jsonrpc":"2.0", "id":1, "method":method, "params": params or {}}
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(5)
        s.connect(path)
        s.sendall(json.dumps(request).encode()+b"\n")
        with s.makefile("rb") as stream:
            line=stream.readline(1_048_577)
    if not line or len(line)>1_048_576: raise RuntimeError("missing or oversized response")
    result=json.loads(line)
    if "error" in result: raise RuntimeError(json.dumps(result["error"], ensure_ascii=False))
    return result["result"]


def main() -> int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=["status","rearm"], nargs="?", default="status")
    p.add_argument("--socket", default="/run/robotd.sock")
    a=p.parse_args()
    method="robot.morphology.rearm" if a.action=="rearm" else "robot.morphology"
    r=rpc(a.socket,method)
    print(json.dumps(r,ensure_ascii=False,indent=2))
    if a.action=="status" and r.get("patch")!="headless-v015-r2":
        raise RuntimeError("wrong/unpatched robotd identity")
    if a.action=="rearm":
        print("Rearm is only queued. Poll status; it must show fault_latched=false before a separate init/Start.")
        return 0 if r.get("accepted") else 2
    return 0

if __name__=="__main__":
    try: raise SystemExit(main())
    except (OSError,RuntimeError,ValueError,KeyError) as e:
        print(f"ERROR: {e}",file=sys.stderr); raise SystemExit(1)
