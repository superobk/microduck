#!/usr/bin/env python3
"""Board-side migration steps. No action starts a hardware daemon.

Invoked only by deploy.py's audited SSH transport. Each write is under a
run-owned directory or an exclusively-created unit/drop-in. Existing official
binary/config/policy files are never overwritten. Rollback retains the two
owned safety holds while head joints are absent; it never restores active state.
Python >=3.11 is required so configuration parsing has the original TOML types.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time
import tomllib
import ctypes
import importlib.util

BASE='/opt/robot/local/zero3w-11servo'
CONFIG=Path('/etc/robot/robotd.toml')
UNITS=['robotd.service','updaterd.service','configd.service','padd.service','btd.service','tofd.service','mediad.service']
HOLD='[Unit]\nConditionPathExists=/etc/robot/ZERO3W_FULL15_RESTORED\n'
UPDATE_HOLD='[Unit]\nConditionPathExists=/etc/robot/ZERO3W_OFFICIAL_UPDATE_REVIEWED\n'
OWNED={Path('/etc/systemd/system/robotd.service.d/91-zero3w-hold.conf'):HOLD,
       Path('/etc/systemd/system/updaterd.service.d/91-zero3w-hold.conf'):UPDATE_HOLD}
CANDIDATE_SERVICE=Path('/etc/systemd/system/zero3w-11servo-candidate.service')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def command(argv,check=True):
    p=subprocess.run(argv,capture_output=True,text=True)
    # Print only selected commands' output. Never print environment or account files.
    print(json.dumps({'command':argv,'exit_code':p.returncode,'stdout':p.stdout,'stderr':p.stderr}),flush=True)
    if check and p.returncode:raise RuntimeError(f'command failed: {argv[0]}')
    return p
def exclusive(p,text):
    expected=hashlib.sha256(text.encode()).hexdigest()
    print(json.dumps({'write_attempt':str(p),'expected_sha256':expected}),flush=True)
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:f.write(text)
    print(json.dumps({'write_completed':str(p),'sha256':sha(p)}),flush=True)
def dump(p,value):exclusive(p,json.dumps(value,indent=2)+'\n')
def snapshot():
    files=[]
    for p in [CONFIG,Path('/etc/robot/updater.toml'),Path('/opt/robot/daemon/current/bin/robotd')]:
        files.append({'path':str(p),'exists':p.is_file(),'resolved':str(p.resolve()),'sha256':sha(p) if p.is_file() else None})
    units={}
    for unit in UNITS:
        result=command(['systemctl','show',unit,'--property=ActiveState,SubState,UnitFileState,FragmentPath,DropInPaths,MainPID'],False)
        units[unit]=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        paths=[units[unit].get('FragmentPath',''),*units[unit].get('DropInPaths','').split()]
        units[unit]['unit_files']=[{'path':name,'resolved':str(Path(name).resolve()),'sha256':sha(name)} for name in paths if name and Path(name).is_file()]
        # Record executable identity of a running daemon without printing argv/env.
        pid=units[unit].get('MainPID','0')
        exe=Path('/proc')/pid/'exe'
        if pid!='0' and exe.is_file():units[unit]['running_exe']={'path':os.readlink(exe),'sha256':sha(exe)}
    params=tomllib.loads(CONFIG.read_text()) if CONFIG.is_file() else {}
    selected={k:params.get(k,{}) for k in ('bus','policy','control','safety')}
    policy_files=[]
    # Path.rglob does not reliably recurse through the deployed current symlink.
    # Inspect its resolved tree explicitly, deduplicating by physical model path.
    for directory in [Path('/opt/robot/policies'),Path('/opt/robot/policies/current').resolve()]:
        for p in sorted(directory.rglob('*.onnx')):
            if p.is_file() and not any(x['resolved']==str(p.resolve()) for x in policy_files):policy_files.append({'path':str(p),'resolved':str(p.resolve()),'sha256':sha(p)})
    runtimes=[]
    for directory in ['/usr/lib','/usr/local/lib','/opt/robot']:
        for p in Path(directory).glob('**/libonnxruntime.so*'):
            if p.is_file() and not any(x['resolved']==str(p.resolve()) for x in runtimes):
                row={'path':str(p),'resolved':str(p.resolve()),'sha256':sha(p)}
                try:
                    # Query the C API's version string, not a guessed soname.
                    version_fn=ctypes.CFUNCTYPE(ctypes.c_char_p)
                    class ApiBase(ctypes.Structure):_fields_=[('get_api',ctypes.c_void_p),('get_version',version_fn)]
                    library=ctypes.CDLL(str(p));library.OrtGetApiBase.restype=ctypes.POINTER(ApiBase)
                    row['runtime_version']=library.OrtGetApiBase().contents.get_version().decode()
                except Exception as error:row['runtime_version_error']=str(error)
                runtimes.append(row)
    manifests=[]
    for p in Path('/opt/robot/daemon/current').glob('**/manifest.json'):
        obj=json.loads(p.read_text());manifests.append({'path':str(p),'sha256':sha(p),'identity':{k:obj.get(k) for k in ('version','revision','commit','git_sha','build')}})
    return {'machine':os.uname().machine,'host':socket.gethostname(),'uid':os.geteuid(),
            'files':files,'units':units,'robot_config_selected':selected,'policy_files':policy_files,
            'onnx_runtime_libraries':runtimes,'daemon_manifests':manifests,
            'actual_onnx_runtime_version':'queried through each library OrtGetApiBase; errors remain explicit'}
def rpc(path,method,params=None):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.settimeout(3);s.connect(str(path));s.sendall(json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params or {}}).encode()+b'\n')
        return json.loads(s.makefile('rb').readline())
def candidate_body(root):
    return f'[Unit]\nDescription=Zero3W fixed-head candidate (stopped)\nConditionPathExists=/etc/robot/ZERO3W_HARDWARE_APPROVED\n[Service]\nType=simple\nEnvironment=MICRODUCK_MORPHOLOGY={root}/morphology.json\nEnvironment=DUCK_RUNTIME_DIR=/run/zero3w-candidate\nExecStart={root}/robotd --params {root}/robotd.toml --socket /run/zero3w-candidate.sock\nRestart=no\n'
def rollback(root,full15_restored=False):
    """Repeatable rollback, including interruption before installed.json exists.

    transaction.json is written BEFORE stopping services or creating systemd
    files. Missing files mean an uncompleted/already undone step, not failure.
    An existing file must match the recorded ownership hash; drift never grants
    permission to erase another operator's work. Headless hardware keeps holds.
    """
    journal=root/'transaction.json'
    manifest=root/'installed.json'
    if not journal.is_file() and not manifest.is_file():
        print(json.dumps({'rollback':'no systemd transaction begun','official_files_preserved':True}));return
    source=journal if journal.is_file() else manifest
    expected=json.loads(source.read_text())['owned_systemd_files']
    allowed={str(f):hashlib.sha256(text.encode()).hexdigest() for f,text in OWNED.items()}
    allowed[str(CANDIDATE_SERVICE)]=hashlib.sha256(candidate_body(root).encode()).hexdigest()
    if expected!=allowed:raise RuntimeError('ownership manifest does not match the exact candidate paths/content')
    for name,want in expected.items():
        f=Path(name)
        if f.is_symlink() or (f.exists() and (not f.is_file() or sha(f)!=want)):
            raise RuntimeError(f'file changed since transaction, manual review required: {f}')
    # Even if the unit file is absent, daemon-reload may not yet have happened
    # after an interrupted rollback. Stop a still-loaded unit before reloading.
    state=command(['systemctl','show','zero3w-11servo-candidate.service','--property=LoadState','--value'])
    if state.stdout.strip() not in ('','not-found'):
        command(['systemctl','stop','zero3w-11servo-candidate.service'])
    if CANDIDATE_SERVICE.exists():CANDIDATE_SERVICE.unlink()
    if full15_restored:
        for f in OWNED:
            if f.exists():f.unlink()
    command(['systemctl','daemon-reload'])
    retained=[str(f) for f in OWNED if f.exists()]
    result={'candidate_service_removed':True,'official_service_restarted':False,
            'operator_full15_restored':full15_restored,'holds_retained_until_full15_restored':retained,
            'transaction_source':str(source),'repeated_call_supported':True}
    dump(root/f'rollback-{time.time_ns()}.json',result);print(json.dumps(result))
def fake(root):
    # No board bus or original runtime socket, no default configuration, no audio.
    # --no-policy deliberately separates protocol acceptance from real ONNX tests.
    with tempfile.TemporaryDirectory(prefix='zero3w-fake-',dir='/tmp') as tmp:
        tmp=Path(tmp);config=tmp/'robotd.toml';sock=tmp/'s'
        config.write_text('[policy]\nenabled=false\n[audio]\nenabled=false\n[safety]\nbattery_empty_shutdown=false\n')
        env=dict(os.environ,MICRODUCK_MORPHOLOGY=str(root/'morphology.json'),DUCK_RUNTIME_DIR=str(tmp/'runtime'))
        logfile=root/'fake-runtime.log'
        with logfile.open('ab') as log:
            p=subprocess.Popen([str(root/'robotd'),'--fake','--no-policy','--params',str(config),'--socket',str(sock)],env=env,stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+15
                while not sock.exists():
                    if p.poll() is not None or time.monotonic()>deadline:raise RuntimeError('fake daemon did not start; inspect fake-runtime.log')
                    time.sleep(.05)
                report=rpc(sock,'robot.morphology')['result']
                while not report['imu_ready']:
                    if time.monotonic()>deadline:raise RuntimeError('fake loop did not publish ready sensors')
                    time.sleep(.05);report=rpc(sock,'robot.morphology')['result']
                health=rpc(sock,'robot.health')['result']
                assert health['healthy'] and health['control_loop']['ticks']>0,health
                # Pinned v0.15.0's HelloParams requires api_version (shared ABI37).
                hello=rpc(sock,'hello',{'api_version':37})['result']
                installed=json.loads((root/'installed.json').read_text())
                assert hello['revision']==installed['source_commit'],'running binary revision differs from candidate manifest'
                assert report['active_motor_ids']==[20,21,22,23,24,34,10,11,12,13,14]
                assert report['policy_observation_width']==61 and report['policy_action_width']==14
                for method,params in [('robot.enable',{'on':True}),('robot.init',{}),('robot.head',{'neck_pitch':.1,'head_pitch':.1,'head_yaw':.1,'head_roll':.1}),('robot.do',{'skill':'roulade'})]:
                    assert rpc(sock,method,params)['result']['accepted'] is False
                result={'kind':'board_isolated_fake_no_bus_no_onnx','pass':True,'hello':hello,'health':health,'morphology':report}
                print(json.dumps(result));dump(root/f'fake-result-{time.time_ns()}.json',result)
            finally:
                p.terminate()
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['preflight','prepare','backup','install','fake','rollback'])
    p.add_argument('run_id');p.add_argument('--sha256');p.add_argument('--source-commit');p.add_argument('--config-helper-sha');p.add_argument('--power-isolated',action='store_true')
    p.add_argument('--full15-restored',action='store_true')
    a=p.parse_args()
    if not a.run_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_+-' for c in a.run_id):raise ValueError('unsafe run ID')
    root=Path(BASE)/a.run_id
    if a.action=='preflight':print(json.dumps(snapshot(),indent=2));return
    if os.geteuid()!=0 or os.uname().machine!='aarch64':raise RuntimeError('need root on AArch64 Linux')
    if a.action=='prepare':
        root.mkdir(parents=True,exist_ok=False);root.chmod(0o700)
        dump(root/'ownership.json',{'run_id':a.run_id,'owner':'zero3w-11servo-candidate','official_files_overwritten':False});return
    if json.loads((root/'ownership.json').read_text())['run_id']!=a.run_id:raise RuntimeError('wrong directory owner')
    if a.action in ('backup','install','rollback') and not a.power_isolated:raise RuntimeError('motor power must be physically isolated by operator before service mutations')
    if a.action=='backup':
        before=snapshot();dump(root/'before.json',before);backup=root/'backup';backup.mkdir()
        for f in [CONFIG,Path('/etc/robot/updater.toml'),Path('/opt/robot/daemon/current/bin/robotd')]:
            if f.is_file():shutil.copy2(f,backup/f.name)
        # Unit files/drop-ins remain private on the board; log only metadata/hashes.
        unitdir=backup/'systemd';unitdir.mkdir()
        for name in UNITS:
            for f in [Path('/etc/systemd/system')/name,Path('/etc/systemd/system')/(name+'.d')]:
                if f.is_dir():shutil.copytree(f,unitdir/f.name)
                elif f.is_file():shutil.copy2(f,unitdir/f.name)
        # Also preserve vendor fragments/effective drop-ins outside /etc. Hash
        # every copy, so a "backup" with changing/unreadable files cannot pass.
        index=[]
        for unit,entry in before['units'].items():
            for i,item in enumerate(entry['unit_files']):
                original=Path(item['path']);dest=unitdir/'effective'/unit/f'{i}-{original.name}'
                dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(original,dest)
                if sha(dest)!=item['sha256']:raise RuntimeError('effective unit changed during backup')
                index.append({**item,'backup_path':str(dest)})
        for item in before['files']:
            if item['exists'] and sha(backup/Path(item['path']).name)!=item['sha256']:raise RuntimeError('official file changed during backup')
        dump(root/'backup-index.json',{'effective_unit_files':index,'official_files':before['files']})
        return
    if a.action=='install':
        if not (root/'before.json').exists():raise RuntimeError('backup required before installation')
        before=json.loads((root/'before.json').read_text())
        for item in before['files']:
            f=Path(item['path'])
            if f.is_file()!=item['exists'] or (f.is_file() and sha(f)!=item['sha256']):raise RuntimeError(f'official file drift since backup: {f}')
        if not (root/'backup-index.json').is_file():raise RuntimeError('verified backup index required')
        for unit in before['units'].values():
            for item in unit['unit_files']:
                f=Path(item['path'])
                if not f.is_file() or sha(f)!=item['sha256']:raise RuntimeError(f'effective unit drift since backup: {f}')
        staged=root/'robotd.staged'
        if not a.sha256 or sha(staged)!=a.sha256:raise RuntimeError('candidate hash mismatch')
        helper=root/'prepare_config.py'
        if not a.config_helper_sha or sha(helper)!=a.config_helper_sha:raise RuntimeError('configuration helper hash mismatch')
        spec=importlib.util.spec_from_file_location('candidate_config',helper);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        prepared=module.prepare(CONFIG.read_text() if CONFIG.is_file() else '',None,False)
        if (root/'robotd').exists():raise RuntimeError('candidate is immutable; use a fresh run ID')
        # Refuse pre-existing markers and drop-ins before stopping any service.
        for marker in ['ZERO3W_FULL15_RESTORED','ZERO3W_OFFICIAL_UPDATE_REVIEWED','ZERO3W_HARDWARE_APPROVED']:
            if (Path('/etc/robot')/marker).exists():raise RuntimeError('unexpected approval marker')
        service=CANDIDATE_SERVICE
        for f in [*OWNED,service]:
            if f.exists() or f.is_symlink():raise RuntimeError(f'owned path already exists: {f}')
        body=candidate_body(root)
        files={str(f):hashlib.sha256(text.encode()).hexdigest() for f,text in {**OWNED,service:body}.items()}
        # Durable rollback intent precedes the FIRST systemd mutation. Complete
        # installs and interrupted installs share the same exact ownership map.
        dump(root/'transaction.json',{'source_commit':a.source_commit,'binary_sha256':a.sha256,
                                     'owned_systemd_files':files,'hardware_service_started':False})
        for unit in ['robotd.service','updaterd.service']:command(['systemctl','stop',unit])
        for f,text in OWNED.items():exclusive(f,text)
        staged.rename(root/'robotd');(root/'robotd').chmod(0o755)
        dump(root/'morphology.json',{'schema_version':1,'profile':'headless','mouth_present':True,'allow_motion':False,'locked_head_rad':[.3491,.3491,0,0]})
        exclusive(root/'robotd.toml',prepared)
        exclusive(service,body)
        for name,want in files.items():
            if sha(name)!=want:raise RuntimeError('systemd file differs from transaction intent')
        dump(root/'installed.json',{'source_commit':a.source_commit,'binary_sha256':a.sha256,'owned_systemd_files':files,'hardware_service_started':False})
        command(['systemctl','daemon-reload']);command(['systemctl','stop','zero3w-11servo-candidate.service'])
        print(json.dumps({'installed':str(root),'hardware_service':'STOPPED','official_binary':'preserved'}));return
    if a.action=='fake':fake(root);return
    if a.action=='rollback':
        rollback(root,a.full15_restored)
if __name__=='__main__':main()
