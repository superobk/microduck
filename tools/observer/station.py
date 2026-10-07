#!/usr/bin/env python3
"""Explicit, standalone lifecycle commands, usable even when the helper is off.

Only manifest-owned observer units are touched. A version change or board reboot
never starts diagnostics. Keep this release's file path to control an older
rollback release too; it does not rely on the older observerctl supporting it.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess

import install
UNIT_DIR=Path('/etc/systemd/system')

def states():
    result={}
    for unit in install.UNITS:
        r=subprocess.run(['systemctl','show',unit+'.service','--property=LoadState,ActiveState,MainPID,UnitFileState'],capture_output=True,text=True,timeout=4)
        if r.returncode:raise RuntimeError('检测站状态查询失败 '+unit)
        result[unit]=dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)
    return result

def boot_links():
    # systemctl disable removes externally linked unit definitions as well as
    # boot links. Delete only our validated target wants/requires links instead.
    return [p for folder in UNIT_DIR.glob('*') if folder.name.endswith(('.wants','.requires'))
            for unit in install.UNITS if (p:=folder/(unit+'.service')).is_symlink()]

def enforce_manual():
    install.ensure_owned_units()
    for path in boot_links():
        expected=install.BASE/'current'/('helper.service' if path.name.endswith('helper.service') else 'web.service')
        if path.resolve()!=expected.resolve():raise RuntimeError('启动链接已外部修改，拒绝覆盖 '+str(path))
        install.log('manual_boot_link_removed',path=str(path),target=os.readlink(path))
        path.unlink()

def status():
    return {'units':states(),'boot_links':[str(p) for p in boot_links()],
        'current':(install.BASE/'current').resolve().name,
        'previous':(install.BASE/'previous').resolve().name if (install.BASE/'previous').is_symlink() else None,
        'startup':'手动命令；安装、回退和开机不自动启动'}

def operate(verb):
    if verb=='status':return status()
    if verb not in ['start','stop']:raise ValueError('检测站仅允许start/stop/status')
    if os.geteuid()!=0:raise RuntimeError('启动/停止检测站需root；状态查询不启动辅助器')
    install.ensure_owned_units();before=install.snapshot()
    install.log('station_started',verb=verb,before=states())
    enforce_manual()
    current=states()
    if verb=='start':
        install.verify_release((install.BASE/'current').resolve())
        if all(r['ActiveState']=='active' and r['MainPID']!='0' for r in current.values()):
            result={'already_running':True,**status()}
        else:
            # Reset only session intent, including for an older rollback release.
            # Fault history and calibration are retained; start never rearms UART.
            from runtime import atomic_json,boot_id
            path=install.DATA/'desired.json'
            wanted=json.loads(path.read_text()) if path.exists() else {}
            atomic_json(path,{**wanted,'boot':boot_id(),'observing':False,'bus_requested':False,'recording':False})
            install.run(['systemctl','stop','duck-observer.service','duck-observer-helper.service'])
            install.run(['systemctl','start','duck-observer.service'])
            try:health=install.wait_healthy()
            except Exception:
                install.run(['systemctl','stop','duck-observer.service','duck-observer-helper.service'])
                raise
            result={**status(),'observing':health['observing'],'bus_requested':health['bus_requested']}
    else:
        install.run(['systemctl','stop','duck-observer.service','duck-observer-helper.service'])
        result=status()
        if any(r['MainPID']!='0' or r['ActiveState'] not in ['inactive','failed'] for r in result['units'].values()):
            raise RuntimeError('检测站尚未停止，查看自有unit日志')
    if install.snapshot()!=before:raise RuntimeError('原服务或文件变化，停止后续操作并保留审计')
    install.log('station_completed',verb=verb,result=result,protected_unchanged=True)
    return {**result,'protected_unchanged':True}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('verb',choices=['start','stop','status'])
    print(json.dumps(operate(p.parse_args().verb),ensure_ascii=False,indent=2))
