#!/usr/bin/env python3
"""Passive Linux UART counters; never opens UART or changes the driver.

Inserted/missing bytes must not be 'repaired' into invented telemetry. Kernel
frame/overrun counters help distinguish byte transport from a UI decoder failure.
"""
import datetime
import json
from pathlib import Path
import platform
import sys
sys.path.insert(0,'/opt/robot/local/duck-observer/current')
from server import request_local
state=request_local('/run/duck-observer/helper.sock','status')
path=Path('/proc/tty/driver/serial')
print(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'kernel':platform.release(),
    'ttyS2':str(Path('/sys/class/tty/ttyS2/device').resolve()),
    'serial_line2':[s for s in path.read_text().splitlines() if s.startswith('2:')] if path.exists() else 'not available',
    'ownership':state['ownership'],'bus_state':state['bus_state'],
    'errors':{k:state[k]['errors'] for k in ['servo','imu','tof']},
    'last_errors':[{k:e.get(k) for k in ['utc','kind','component','reason']} for e in state['events'] if e['kind']=='component_error'][-6:]},ensure_ascii=False,indent=2))
