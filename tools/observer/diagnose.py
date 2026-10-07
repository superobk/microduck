#!/usr/bin/env python3
"""Passive own-service diagnostics, always execute with audited_script.py.

No device is opened. A listener and actual HTTP response are separate from a
successful systemctl start; retain startup failures without hiding them.
"""
import json
import subprocess
from urllib.request import build_opener,ProxyHandler
rows=[]
for argv in [
    ['systemctl','status','duck-observer.service','duck-observer-helper.service','--no-pager','--full'],
    ['journalctl','-u','duck-observer.service','-u','duck-observer-helper.service','-n','60','--no-pager'],
    ['ss','-ltnp'],['fuser','/dev/ttyS2']]:
    r=subprocess.run(argv,capture_output=True,text=True,timeout=10)
    rows.append({'argv':argv,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
try:
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8766/api/status',timeout=4) as r:
        state=json.load(r)
    rows.append({'http_status':200,'bus_state':state['bus_state'],'servo_error':state['servo'].get('error'),'tof_error':state['tof'].get('error')})
except Exception as exc:rows.append({'http_error':str(exc)})
print(json.dumps(rows,ensure_ascii=False,indent=2))
