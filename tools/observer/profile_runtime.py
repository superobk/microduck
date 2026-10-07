#!/usr/bin/env python3
"""Passive timing evidence. Never opens a device or changes a service."""
import json
from pathlib import Path
import statistics
import sys
import time
import subprocess
sys.path.insert(0,'/opt/robot/local/duck-observer/current')
from runtime import serial_owners
from server import request_local
cost=[]
for _ in range(15):
    start=time.monotonic();owners=serial_owners('/dev/ttyS2');cost.append((time.monotonic()-start)*1000)
report=request_local('/run/duck-observer/helper.sock','report')
native=[]
for _ in range(10):
    start=time.monotonic();r=subprocess.run(['fuser','/dev/ttyS2'],capture_output=True,text=True,timeout=1);native.append((time.monotonic()-start)*1000)
counts=[]
for proc in Path('/proc').glob('[0-9]*'):
    try:counts.append((len(list((proc/'fd').iterdir())),int(proc.name),(proc/'comm').read_text().strip()))
    except OSError:pass
print(json.dumps({'ownership_check_ms':cost,'median_ms':statistics.median(cost),'owners':owners,
    'fd_largest':sorted(counts,reverse=True)[:10],'resources':report['summary'][-1]['resources'],
    'native_fuser_ms':native,'imu':{k:report['status']['imu'].get(k) for k in ['hz','status','age_ms','errors']}},ensure_ascii=False,indent=2))
