#!/usr/bin/env python3
"""Read-only acceptance window; ending it never stops the observation session.

Use observerctl through the SSH audit wrapper. One sample per second is sufficient
for stability/resource trends; the helper's 50Hz raw stream has its own recorder.
Rolling bus p99 covers 3000 transactions, not an invented aggregate percentile.
"""
import argparse
import csv
import json
from pathlib import Path
import re
import statistics
import time

from server import request_local
from install import DATA,snapshot
from runtime import atomic_json,utc

def brief(state):
    return {k:state.get(k) for k in ['version','uptime_s','bus_state','bus_requested','bus_target_hz','diagnostic_exempt_ids','bus_p99_ms','queue_sizes','record_drops','recording','record_bytes','audio','kernel_uart']} | {
        'components':{k:{x:state[k].get(x) for x in ['status','hz','age_ms','errors','error']} for k in ['servo','imu','tof','camera']},
        'devices':state['servo']['devices'],
        'services':{k:{x:r.get(x) for x in ['ActiveState','MainPID']} for k,r in state['services'].items()}}

def continuity(initial,current):
    # Wall clocks may correct after a board reboot. Boot identity and monotonic
    # helper uptime prevent stitching two sessions/releases into a claimed pass.
    if initial.get('boot')!=current.get('boot'):return '板端已重启，连续窗口中断'
    if initial.get('version')!=current.get('version'):return '门户版本已改变，连续窗口中断'
    if current['uptime_s']<initial['uptime_s']:return '辅助器已重启，连续窗口中断'
    return None

def run(duration=1800,label='first-30min',sock='/run/duck-observer/helper.sock'):
    if not 60<=duration<=86400 or not re.fullmatch(r'[A-Za-z0-9_-]{1,60}',label):raise ValueError('验收窗口60–86400秒，标签只能是字母、数字、横线')
    root=DATA/'acceptance'/label;root.mkdir(parents=True,exist_ok=False)
    before=snapshot();atomic_json(root/'before.json',before)
    start=time.monotonic();rows=[];initial=None;first_done=False;tick=0;interruption=None
    columns=['utc','elapsed_s','version','uptime_s','servo_hz','imu_hz','tof_hz','camera_fps','p99_ms','servo_errors','imu_errors','tof_errors','rss_kib','cpu_percent','disk_free','record_drops','action_queue','record_queue','servo_status','imu_status','tof_status','camera_status']
    with (root/'samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader()
        while time.monotonic()-start<=duration:
            elapsed=time.monotonic()-start
            try:
                state=request_local(sock,'status');report=request_local(sock,'report');resources=report['summary'][-1]['resources'] if report['summary'] else {}
                if initial is None:initial=state;atomic_json(root/'initial.json',brief(state))
                row={'utc':utc(),'elapsed_s':elapsed,'version':state['version']['version'],'uptime_s':state['uptime_s'],
                    **{k+'_hz':state[k]['hz'] for k in ['servo','imu','tof']},'camera_fps':state['camera'].get('stats',{}).get('fps'),
                    'p99_ms':state['bus_p99_ms'],**{k+'_errors':state[k]['errors'] for k in ['servo','imu','tof']},
                    'rss_kib':int(resources.get('VmRSS','0 kB').split()[0]),'cpu_percent':resources.get('cpu_percent'),
                    'disk_free':resources.get('disk_free'),'record_drops':state['record_drops'],
                    'action_queue':state['queue_sizes']['action'],'record_queue':state['queue_sizes']['record'],
                    **{k+'_status':state[k]['status'] for k in ['servo','imu','tof','camera']}}
                writer.writerow(row);f.flush();rows.append(row)
                interruption=continuity(initial,state)
                if interruption:break
            except Exception as exc:
                with (root/'errors.jsonl').open('a') as err:err.write(json.dumps({'utc':utc(),'error':str(exc)})+'\n')
            if elapsed>=60 and not first_done and rows:
                atomic_json(root/'first60.json',evaluate(rows,initial,state,60));first_done=True
            if int(elapsed)%60==0:print(json.dumps({'elapsed_s':round(elapsed),'samples':len(rows),'latest':rows[-1] if rows else None},ensure_ascii=False),flush=True)
            tick+=1;time.sleep(max(.01,start+tick-time.monotonic()))
    if initial is None:raise RuntimeError('验收期间未取得任何有效状态；查看errors.jsonl，不生成通过结论')
    final=request_local(sock,'status');after=snapshot();atomic_json(root/'after.json',after)
    result=evaluate(rows,initial,final,duration);result.update(protected_unchanged=before==after,ended_utc=utc(),observer_continues=final['observing'],evidence=str(root),kernel_uart_initial=initial.get('kernel_uart'),kernel_uart_final=final.get('kernel_uart'))
    interruption=interruption or continuity(initial,final)
    if interruption:result.update(interruption=interruption,ordinary_bus_pass=False,diagnostic_bus_pass=False,imu_only_pass=False,same_helper_process=False)
    atomic_json(root/'final.json',result);atomic_json(root/'last.json',brief(final))
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True);return result

def evaluate(rows,initial,final,duration):
    usable=rows[10:];values=lambda k:[r[k] for r in usable if isinstance(r.get(k),(int,float))]
    mean=lambda k:statistics.mean(values(k)) if values(k) else None
    maximum=lambda k:max(values(k)) if values(k) else None
    stale={k:sum(r[k+'_status']!='实时' for r in usable) for k in ['servo','imu','tof','camera']}
    deltas={k:final[k]['errors']-initial[k]['errors'] for k in ['servo','imu','tof']}
    rss_start=statistics.median([r['rss_kib'] for r in rows[30:90]]) if len(rows)>90 else None
    rss_end=statistics.median([r['rss_kib'] for r in rows[-60:]]) if rows else None
    growth=None if rss_start is None or rss_end is None else rss_end-rss_start
    hz=mean('servo_hz');p99=maximum('p99_ms')
    connected=sum(d.get('connected') is True for d in final['servo']['devices'].values())
    exempt=final.get('diagnostic_exempt_ids',[])
    required=11-len(exempt); target=final.get('bus_target_hz',50)
    diagnostic_pass=bool(connected==required and hz and hz>=target*.9 and p99 is not None and p99<20 and not stale['servo'] and not stale['imu'] and not deltas['servo'] and not deltas['imu'])
    return {'window_s':duration,'samples':len(rows),'sample_span_s':rows[-1]['elapsed_s']-rows[0]['elapsed_s'] if rows else 0,
        'average_servo_hz':hz,'average_imu_hz':mean('imu_hz'),'average_tof_hz':mean('tof_hz'),'average_camera_capture_fps':mean('camera_fps'),
        'max_rolling_bus_p99_ms':p99,'error_delta':deltas,'non_live_samples_after_warmup':stale,
        'cpu_average_percent':mean('cpu_percent'),'cpu_peak_percent':maximum('cpu_percent'),
        'rss_start_kib':rss_start,'rss_end_kib':rss_end,'rss_growth_kib':growth,
        'queue_max':{'action':maximum('action_queue'),'record':maximum('record_queue')},'record_drops':final['record_drops']-initial['record_drops'],
        'same_helper_process':final['uptime_s']>=initial['uptime_s']+max(0,duration-3),
        'connected_servos':connected,'imu_only_pass':bool(mean('imu_hz') and mean('imu_hz')>=45 and p99 is not None and p99<20 and not stale['imu'] and not deltas['imu']),
        'required_servos':required,'diagnostic_exempt_ids':exempt,'bus_target_hz':target,'diagnostic_bus_pass':diagnostic_pass,
        'ordinary_bus_pass':bool(not exempt and connected==11 and hz and hz>=45 and p99 is not None and p99<20 and not stale['servo'] and not stale['imu'] and not deltas['servo'] and not deltas['imu']),
        'limits':'ID14默认OK仅为用户诊断豁免，不能计实测连接；20Hz诊断不计50Hz全链通过；采集统计与浏览器实际解码分别验收；RSS为趋势证据；结束窗口不停止采集'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--duration',type=int,default=1800);p.add_argument('--label',default='first-30min');a=p.parse_args();run(a.duration,a.label)
