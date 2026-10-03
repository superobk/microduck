#!/usr/bin/env python3
"""Generate a conservatively configured COPY of robotd.toml (Python >= 3.11).
Never rewrites the original. Existing ONNX paths/gains/filters are preserved unless
--walk is explicitly given. The hardware topology lives in the separate JSON file.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import json
import re
import sys
import tomllib


def set_scalar(text: str, section: str, key: str, value: str) -> str:
    """Edit a top-level scalar section, preserving unrelated text and comments."""
    # Keep an insertion at EOF from being glued to a last line without a newline.
    if text and not text.endswith("\n"): text += "\n"
    lines=text.splitlines(True)
    starts=[i for i,l in enumerate(lines) if re.match(r"^\s*\["+re.escape(section)+r"\]\s*(?:#.*)?$",l.rstrip("\n"))]
    if len(starts)>1: raise ValueError(f"duplicate section {section}")
    replacement=f"{key} = {value}\n"
    if not starts:
        return text.rstrip()+f"\n\n[{section}]\n"+replacement
    start=starts[0]
    end=next((i for i in range(start+1,len(lines)) if re.match(r"^\s*\[",lines[i])),len(lines))
    keys=[i for i in range(start+1,end) if re.match(r"^\s*"+re.escape(key)+r"\s*=",lines[i])]
    if len(keys)>1: raise ValueError(f"duplicate {section}.{key}")
    if keys: lines[keys[0]]=replacement
    else: lines.insert(end,replacement)
    return "".join(lines)


def prepare(text: str, walk: str | None, regulated: bool, plain_sync: bool=False) -> str:
    original=tomllib.loads(text)
    result=text
    settings={"control":{"hz":"50"},"policy":{"mode":"\"walk\""},"safety":{"limp_fall":"false"}}
    for name in ("sitstand","ground_pick","kick_left","kick_right","roulade"):
        settings["policy"][name]='"none"'
    if plain_sync: settings["bus"]={"fast_sync_read":"false"}
    if walk is not None: settings["policy"]["walk"]=json.dumps(walk)
    if regulated:
        settings["policy"]["voltage_adapt"]="false"
        settings["safety"]["battery_empty_shutdown"]="false"
    for section,items in settings.items():
        for key,value in items.items(): result=set_scalar(result,section,key,value)
    parsed=tomllib.loads(result)
    # The patch must not silently retune the original ONNX controller.
    for key in ("stand","action_scale","standing_action_scale","standing_gain_ratio","gain","head_lowpass","legs_lowpass"):
        assert parsed.get("policy",{}).get(key)==original.get("policy",{}).get(key), key
    if walk is None:
        assert parsed.get("policy",{}).get("walk")==original.get("policy",{}).get("walk")
    return result


def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--walk",help="existing ONNX absolute path on the board; not downloaded or modified")
    ap.add_argument("--regulated-5v",action="store_true",help="disable assumptions that servo voltage equals the 2S battery voltage")
    ap.add_argument("--plain-sync-read",action="store_true",help="use ordinary Sync Read for firmware without Fast Sync Read")
    a=ap.parse_args()
    if a.input.resolve()==a.output.resolve(): raise ValueError("output must differ from input")
    text=a.input.read_text()
    new=prepare(text,a.walk,a.regulated_5v,a.plain_sync_read)
    with a.output.open("x") as f: f.write(new)
    print(f"Wrote {a.output}. Review the diff before installing it. No service was restarted.")
    return 0

if __name__=="__main__":
    try: raise SystemExit(main())
    except (OSError,ValueError,AssertionError) as e:
        print(f"ERROR: {e}",file=sys.stderr); raise SystemExit(1)
