#!/usr/bin/env python3
"""Bounded, independent, read-only packet evidence after observe off.

Compare IMU-only and full-chain rates without another UART owner. Preserve the
first failed exchanges verbatim; never relax CRC, fabricate samples, or change
registers to make a diagnostic pass. Restore termios on every exit.
"""
import json
import os
import statistics
import sys
import time
sys.path.insert(0,'/opt/robot/local/duck-observer/current')
import protocol as p
from runtime import service_state,serial_owners

def guard():
    states={n:service_state(n) for n in ['robotd','zero3w-11servo-candidate']}
    if any(not r.get('query_ok') or r.get('MainPID')!='0' or r.get('ActiveState') not in ['inactive','failed'] for r in states.values()):
        raise RuntimeError('控制器未确认停止；拒绝独立诊断')

guard()
if serial_owners('/dev/ttyS2'):raise RuntimeError('UART仍被占用；先停止门户采集')
port=p.ReadOnlyPort('/dev/ttyS2');results=[];native_read=os.read
try:
    for name,hz,ids,count,slow in [('imu-50hz',50,[200],100,False),('full-20hz',20,p.BUS_IDS,100,False),('full-50hz',50,p.BUS_IDS,200,False),('full-50hz-slow',50,p.BUS_IDS,200,True)]:
        errors=[];latency=[];success=0;guard_at=0.;target=time.monotonic();start=target
        for index in range(count):
            if time.monotonic()-guard_at>.5:guard();guard_at=time.monotonic()
            wire=bytearray()
            def traced_read(fd,size):
                data=native_read(fd,size)
                if fd==port.fd:wire.extend(data)
                return data
            os.read=traced_read;begin=time.monotonic()
            try:
                port.sync(ids=ids)
                if slow and index%5==0:
                    ident=p.SERVO_IDS[(index//5)%11]
                    port.read(ident,144,3);port.read(ident,64,7)
                success+=1  # only a complete transaction counts, including slow reads
            except Exception as exc:
                if len(errors)<12:errors.append({'sample':index,'reason':str(exc),'sync_request_hex':p.sync_request(ids=ids).hex(),'rx_hex':wire.hex()})
            finally:os.read=native_read
            latency.append((time.monotonic()-begin)*1000)
            target+=1/hz;time.sleep(max(0,target-time.monotonic()))
        ordered=sorted(latency)
        row={'phase':name,'requested_hz':hz,'samples':count,'success':success,'elapsed_s':time.monotonic()-start,
            'mean_transaction_ms':statistics.mean(latency),'p99_transaction_ms':ordered[min(len(ordered)-1,int(len(ordered)*.99))],'first_errors':errors}
        results.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
finally:
    os.read=native_read
    port.close()
print(json.dumps({'results':results,'register_writes':0,'port_restored':True},ensure_ascii=False,indent=2))
