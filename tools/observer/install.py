#!/usr/bin/env python3
"""Independent, manifest-owned deployment. Never uses official update hooks.

The official installer starts all shipped daemons; using it for an observation
portal would defeat the held 11-servo controller boundary. Only our two units,
links and version directories may be written. Perception units are left intact.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile
import time
import uuid

BASE=Path('/opt/robot/local/duck-observer')
DATA=Path('/var/lib/duck-observer')
UNITS=['duck-observer-helper','duck-observer']
PROTECTED=['robotd','zero3w-11servo-candidate','updaterd','tofd','mediad']

def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def log(kind,**data):
    (DATA/'audit').mkdir(parents=True,exist_ok=True)
    row={'id':uuid.uuid4().hex,'utc':dt.datetime.now(dt.timezone.utc).isoformat(),'kind':kind,**data}
    p=DATA/'audit'/('events-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d')+'.jsonl')
    with p.open('a') as f: f.write(json.dumps(row,ensure_ascii=False)+'\n');f.flush();os.fsync(f.fileno())
    os.chmod(p,0o640)

def run(args):
    log('command_started',argv=args)
    r=subprocess.run(args,capture_output=True,text=True,timeout=30)
    log('command_finished',argv=args,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr)
    if r.returncode: raise RuntimeError(r.stderr or 'command failed')
    return r.stdout

def snapshot():
    result={}
    for unit in PROTECTED:
        r=subprocess.run(['systemctl','show',unit+'.service','--property=ActiveState,MainPID,UnitFileState,FragmentPath,DropInPaths'],capture_output=True,text=True,timeout=4)
        if r.returncode: raise RuntimeError('无法读取受保护服务 '+unit)
        state=dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)
        state['hashes']={p:digest(p) for p in [state.get('FragmentPath','')]+state.get('DropInPaths','').split() if p and Path(p).is_file()}
        result[unit]=state
    files=['/etc/robot/robotd.toml']
    for root in ['/opt/robot/daemon/current/bin','/opt/robot/policies/current','/opt/robot/detector/current']:
        p=Path(root)
        if p.exists(): files.extend(str(f) for f in p.rglob('*') if f.is_file())
    result['files']={f:digest(f) for f in sorted(files) if Path(f).is_file()}
    return result

def check_archive(path,sha):
    if digest(path)!=sha: raise RuntimeError('bundle SHA256不符')
    with tarfile.open(path,'r:gz') as tar:
        total=0
        for item in tar.getmembers():
            parts=Path(item.name).parts
            if Path(item.name).is_absolute() or '..' in parts or not parts or item.issym() or item.islnk() or not (item.isfile() or item.isdir()):
                raise RuntimeError('拒绝非普通文件或越界归档')
            total+=item.size
            if total>10*1024*1024: raise RuntimeError('bundle异常过大')
        names=[m.name for m in tar.getmembers()]
        if len(names)!=len(set(names)): raise RuntimeError('归档重复路径')

def verify_release(path):
    manifest=json.loads((path/'MANIFEST.json').read_text())
    listed=manifest['files']; actual={str(p.relative_to(path)) for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts}-{'MANIFEST.json'}
    if actual!=set(listed): raise RuntimeError('版本文件清单不一致')
    for name,expected in listed.items():
        p=path/name
        if not p.resolve().is_relative_to(path.resolve()) or p.is_symlink() or digest(p)!=expected: raise RuntimeError('版本文件漂移 '+name)
    return manifest

def link_current(path):
    tmp=BASE/('.current-'+uuid.uuid4().hex)
    tmp.symlink_to(path);os.replace(tmp,BASE/'current')

def ensure_owned_units():
    for unit in UNITS:
        p=Path('/etc/systemd/system')/(unit+'.service')
        target=str(BASE/'current'/('helper.service' if unit.endswith('helper') else 'web.service'))
        if not p.is_symlink() or os.readlink(p)!=target: raise RuntimeError('自有unit已发生外部变化 '+str(p))

def wait_healthy():
    """Type=simple becoming active precedes socket binding; verify the real route."""
    import sys
    sys.path.insert(0,str(BASE/'current'))
    from server import request_local
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        try:
            state=request_local('/run/duck-observer/helper.sock','status')
            from urllib.request import build_opener,ProxyHandler
            with build_opener(ProxyHandler({})).open('http://127.0.0.1:8766/api/status',timeout=2) as r:
                web=json.load(r)
            if web['version']!=state['version']:raise RuntimeError('Web与辅助器版本不同')
            return state
        except (OSError,RuntimeError): time.sleep(.2)
    raise RuntimeError('门户辅助器未在12秒内响应；恢复自有版本，不启动控制器')

def switch_release(verb,version=None,previous=False):
    if os.geteuid()!=0: raise RuntimeError('版本切换需通过审计的root CLI执行')
    ensure_owned_units();before=snapshot();old=(BASE/'current').resolve()
    if verb=='rollback':
        if not previous or not (BASE/'previous').is_symlink(): raise RuntimeError('没有上一版本；首次安装可用release uninstall停用门户')
        target=(BASE/'previous').resolve()
    else:
        if not version or not re.fullmatch(r'[A-Za-z0-9._-]{1,80}',version): raise ValueError('版本名称错误')
        target=BASE/'releases'/version
    if not target.is_relative_to(BASE/'releases'): raise RuntimeError('版本链接越界')
    manifest=verify_release(target)
    tx={'old':str(old),'target':str(target),'verb':verb,'before':before}
    log('release_started',**tx)
    run(['systemctl','stop','duck-observer.service','duck-observer-helper.service'])
    try:
        link_current(target);run(['systemctl','daemon-reload']);run(['systemctl','start','duck-observer.service'])
        wait_healthy()
        after=snapshot()
        if after!=before: raise RuntimeError('原程序/服务发生变化；需查看审计')
        (BASE/'previous').unlink(missing_ok=True);(BASE/'previous').symlink_to(old)
        log('release_completed',version=manifest['version'],before=before,after=after)
        return {'version':manifest['version'],'previous':old.name,'controller_changes':False}
    except Exception:
        log('release_failed',old=str(old),target=str(target))
        run(['systemctl','stop','duck-observer.service','duck-observer-helper.service'])
        link_current(old);run(['systemctl','daemon-reload']);run(['systemctl','start','duck-observer.service']);wait_healthy()
        log('release_recovered',version=old.name,protected_after=snapshot());raise

def install(bundle,sha,version):
    if os.geteuid()!=0: raise RuntimeError('需要root安装独立unit')
    if not re.fullmatch(r'[A-Za-z0-9._-]{1,80}',version): raise ValueError('版本名称错误')
    check_archive(bundle,sha)
    BASE.mkdir(parents=True,exist_ok=True);(BASE/'releases').mkdir(exist_ok=True)
    before=snapshot();dest=BASE/'releases'/version
    if dest.exists(): raise RuntimeError('版本目录已存在，拒绝覆盖')
    with tempfile.TemporaryDirectory(prefix='observer-stage-',dir=BASE) as directory:
        stage=Path(directory)
        # Python 3.11 on the board may predate extractall(filter=). Members have
        # already passed our stricter regular-file/path/size validation above.
        with tarfile.open(bundle,'r:gz') as tar: tar.extractall(stage)
        manifest=verify_release(stage)
        if manifest['version']!=version: raise RuntimeError('版本身份不符')
        subprocess.run(['python3','-m','compileall','-q',str(stage)],check=True)
        # Compile cache is transient, not part of the immutable release identity.
        for p in stage.rglob('__pycache__'): shutil.rmtree(p)
        os.rename(stage,dest)
        # TemporaryDirectory is 0700. After rename the immutable release must be
        # readable/traversable by the deliberately unprivileged web account.
        os.chmod(dest,0o755)
        for p in dest.rglob('*'):
            os.chmod(p,0o755 if p.is_dir() or p.name=='observerctl' else 0o644)
    DATA.mkdir(parents=True,exist_ok=True)
    import pwd,grp
    try: account=pwd.getpwnam('duck-observer')
    except KeyError:
        run(['useradd','--system','--user-group','--home-dir','/nonexistent','--shell','/usr/sbin/nologin','duck-observer']);account=pwd.getpwnam('duck-observer')
    group=grp.getgrnam('duck-observer')
    os.chown(DATA,0,group.gr_gid);os.chmod(DATA,0o2770)
    for p in DATA.rglob('*'):
        if p.is_dir(): os.chown(p,0,group.gr_gid);os.chmod(p,0o2750)
        elif p.is_file(): os.chown(p,0,group.gr_gid);os.chmod(p,0o640)
    backup=BASE/'backups'/version;backup.mkdir(parents=True,exist_ok=False);os.chmod(backup,0o700)
    (backup/'before.json').write_text(json.dumps(before,ensure_ascii=False,indent=2))
    paths=set(before['files'])
    for unit in PROTECTED: paths.update(before[unit]['hashes'])
    for path in paths:
        target=backup/'files'/path.lstrip('/');target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    (dest/'helper.service').write_text(f'''[Unit]
Description=Duck Observer private hardware helper
After=local-fs.target
[Service]
User=root
Group=duck-observer
ExecStart=/usr/bin/python3 {BASE}/current/server.py --helper --web-uid {account.pw_uid}
Restart=on-failure
RestartSec=3
UMask=0007
RuntimeDirectory=duck-observer
RuntimeDirectoryMode=0750
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
ReadWritePaths={DATA} /run/duck-observer
[Install]
WantedBy=multi-user.target
''')
    (dest/'web.service').write_text(f'''[Unit]
Description=Duck Observer LAN portal
Requires=duck-observer-helper.service
After=duck-observer-helper.service
[Service]
User=duck-observer
Group=duck-observer
ExecStart=/usr/bin/python3 {BASE}/current/server.py
Restart=on-failure
RestartSec=3
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
PrivateDevices=yes
[Install]
WantedBy=multi-user.target
''')
    for name in ['helper.service','web.service']:manifest['files'][name]=digest(dest/name)
    (dest/'MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
    first=not (BASE/'current').exists()
    if first:
        for unit in UNITS:
            p=Path('/etc/systemd/system')/(unit+'.service')
            if p.exists() or p.is_symlink(): raise RuntimeError('已有非本工具管理的unit，拒绝覆盖')
        link_current(dest)
        for unit in UNITS:
            target=BASE/'current'/('helper.service' if unit.endswith('helper') else 'web.service')
            Path('/etc/systemd/system',unit+'.service').symlink_to(target)
        log('install_started',version=version,bundle_sha256=sha,protected_before=before)
        try:
            run(['systemctl','daemon-reload']);run(['systemctl','enable','duck-observer.service']);run(['systemctl','start','duck-observer.service'])
            wait_healthy()
            if snapshot()!=before: raise RuntimeError('原服务/文件变化；保留备份，不自动恢复原控制器')
        except Exception:
            # First install has no previous release. Remove only the two links we
            # created; leave every release, backup, journal and original daemon.
            log('first_install_failed',version=version,protected_after=snapshot())
            run(['systemctl','disable','--now','duck-observer.service'])
            run(['systemctl','stop','duck-observer-helper.service'])
            ensure_owned_units()
            for unit in UNITS: Path('/etc/systemd/system',unit+'.service').unlink()
            (BASE/'current').unlink();run(['systemctl','daemon-reload'])
            log('first_install_disabled',version=version);raise
    else: switch_release('activate',version)
    owned={str(Path('/etc/systemd/system')/(u+'.service')):os.readlink(Path('/etc/systemd/system')/(u+'.service')) for u in UNITS}
    (BASE/'OWNED.json').write_text(json.dumps({'units':owned,'data_dir':str(DATA),'created_account':'duck-observer'},indent=2))
    after=snapshot()
    log('install_completed',version=version,protected_after=after)
    if before!=after: raise RuntimeError('受保护服务或原文件变化；禁止宣称安装验收通过')
    return {'installed':version,'port':8766,'protected_unchanged':True,'backup':str(backup)}

def uninstall():
    if os.geteuid()!=0: raise RuntimeError('需要root')
    ensure_owned_units();before=snapshot();log('uninstall_started',before=before)
    run(['systemctl','disable','--now','duck-observer.service']);run(['systemctl','stop','duck-observer-helper.service'])
    for unit in UNITS: Path('/etc/systemd/system',unit+'.service').unlink()
    run(['systemctl','daemon-reload']);after=snapshot()
    if before!=after: raise RuntimeError('原服务变化；审计保留')
    log('uninstall_completed',after=after)
    return {'disabled':True,'preserved':'所有版本、备份、校正、审计和原服务'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path);p.add_argument('--sha256');p.add_argument('--version');p.add_argument('--uninstall',action='store_true');a=p.parse_args()
    print(json.dumps(uninstall() if a.uninstall else install(a.bundle,a.sha256,a.version),ensure_ascii=False,indent=2))
