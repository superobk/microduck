#!/usr/bin/env python3
"""Bounded independent Ping comparison after collector released its UART.

This separates a disconnected physical chain from dashboard decoding. It reads
only fixed IDs and closes/restores the UART even when every device is absent.
"""
import json
import sys
sys.path.insert(0,'/opt/robot/local/duck-observer/current')
import protocol as p
from runtime import service_state,serial_owners
states={n:service_state(n) for n in ['robotd','zero3w-11servo-candidate']}
if any(not r.get('query_ok') or r.get('MainPID')!='0' or r.get('ActiveState') not in ['inactive','failed'] for r in states.values()):raise RuntimeError('控制器未确认停止')
if serial_owners('/dev/ttyS2'):raise RuntimeError('UART仍被占用')
port=p.ReadOnlyPort('/dev/ttyS2');rows=[]
try:
    for ident in p.BUS_IDS:
        try:
            raw,status=port.identity(ident)
            rows.append({'id':ident,'model':int.from_bytes(raw[:2],'little'),'firmware':raw[2],'status':status})
        except Exception as exc:rows.append({'id':ident,'error':str(exc)})
finally:port.close()
print(json.dumps({'ids':rows,'register_writes':0,'port_restored':True},ensure_ascii=False,indent=2))
