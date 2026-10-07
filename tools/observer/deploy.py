#!/usr/bin/env python3
"""Build a small immutable bundle and deploy through the existing audit wrapper.

No official hooks, model updates, motor daemon activation or service restarts are
hidden in this transport. Exact scripts, bundle identity and SSH/SCP failures are
retained locally; CLI operations have the same board-side journal as the GUI.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'zero3w'))
from audit_command import run

def package(version,destination):
    if not re.fullmatch(r'[A-Za-z0-9._-]{1,80}',version): raise ValueError('版本名称错误')
    # A commit label must describe the bytes actually shipped. Reject local
    # modifications and preserve an earlier artifact rather than relabel it.
    dirty=subprocess.check_output(['git','status','--porcelain','--untracked-files=all','--','tools/observer'],cwd=HERE.parents[1],text=True)
    if dirty.strip():raise RuntimeError('门户源码未提交；先审查并提交再生成发布包')
    if destination.exists():raise RuntimeError('候选包已存在；保留证据并使用新的版本名称')
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=HERE,text=True).strip()
    with tempfile.TemporaryDirectory() as directory:
        stage=Path(directory)
        for source in HERE.iterdir():
            if source.suffix=='.py' or source.name=='observerctl': shutil.copy2(source,stage/source.name)
        shutil.copytree(HERE/'static',stage/'static')
        shutil.copy2(HERE.parents[1]/'LICENSE',stage/'LICENSE')
        metadata={'version':version,'revision':revision,'base':'a9ec4b2079ef8ee7904014089c885bb07d57d63c',
            'port':8766,'asset_sha256':hashlib.sha256((stage/'static/duck.bin').read_bytes()).hexdigest(),
            'startup':'manual','default_bus_hz':20,'diagnostic_exempt_ids':[14],
            'servo_writes':False,'controller_lifecycle':False,'sound_seed':114,'sound_generator':'sounds render chirp --seed 114'}
        (stage/'VERSION.json').write_text(json.dumps(metadata,indent=2)+'\n')
        files={str(p.relative_to(stage)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(stage.rglob('*')) if p.is_file()}
        (stage/'MANIFEST.json').write_text(json.dumps({'version':version,'files':files},indent=2)+'\n')
        with tarfile.open(destination,'w:gz') as tar:
            for p in sorted(stage.rglob('*')):
                if p.is_file(): tar.add(p,arcname=str(p.relative_to(stage)))
    return {'version':version,'revision':revision,'path':str(destination),'sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['package','install','cli','export-audit']);p.add_argument('--target')
    p.add_argument('--version',default='');p.add_argument('--tool-version',default='');p.add_argument('--audit-root',type=Path,required=True);a,extra=p.parse_known_args();a.args=extra
    if a.tool_version and (a.action!='cli' or not re.fullmatch(r'[A-Za-z0-9._-]{1,80}',a.tool_version)):p.error('tool-version仅用于指定已安装CLI版本')
    if extra and a.action!='cli':p.error('不允许额外参数')
    archive=a.audit_root.parent/'artifacts';archive.mkdir(exist_ok=True)
    if a.action in ['package','install']:
        if not re.fullmatch(r'[A-Za-z0-9._-]{1,80}',a.version):p.error('版本名称错误')
        bundle=archive/(a.version+'.tar.gz');manifest=archive/(a.version+'.json')
        if a.action=='install' and bundle.exists():
            # Review/package may happen while the board is off. Install those
            # exact bytes later; never rebuild an already-reviewed version.
            meta=json.loads(manifest.read_text())
            if meta.get('version')!=a.version or meta.get('sha256')!=hashlib.sha256(bundle.read_bytes()).hexdigest():
                raise RuntimeError('既有候选包身份/哈希不符；停止安装并保留证据')
            with tarfile.open(bundle,'r:gz') as tar:
                identity=json.load(tar.extractfile('VERSION.json'))
            if identity.get('revision')!=meta.get('revision') or identity.get('version')!=a.version:
                raise RuntimeError('候选包源码身份不符')
        else:
            meta=package(a.version,bundle)
            manifest.write_text(json.dumps(meta,indent=2)+'\n')
        print(json.dumps(meta))
        if a.action=='package': return
    if not a.target or a.target.startswith('-') or any(c.isspace() for c in a.target): p.error('需要确认的SSH目标')
    options=['-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=2']
    if a.action=='install':
        remote='/tmp/duck-observer-'+meta['sha256']+'.tar.gz'
        if run(['scp',*options,str(bundle),a.target+':'+remote],a.audit_root,'上传门户独立候选包','哈希校验后只安装自有版本，不运行官方更新钩子'): raise SystemExit(1)
        data=(HERE/'install.py').read_bytes();digest=hashlib.sha256(data).hexdigest()
        scripts=a.audit_root.parent/'scripts'/digest;scripts.mkdir(exist_ok=True);shutil.copy2(HERE/'install.py',scripts/'install.py')
        loader='import base64,sys;sys.argv='+repr(['install.py','--bundle',remote,'--sha256',meta['sha256'],'--version',a.version])+";exec(compile(base64.b64decode("+repr(base64.b64encode(data).decode())+"),'audited-observer-install.py','exec'))"
        code=run(['ssh',*options,a.target,shlex.join(['python3','-c',loader])],a.audit_root,'安装独立门户文件；备份原状态，保持原服务和文件','安装不启动检测站；需明确station start；保留失败和恢复路径')
    elif a.action=='cli':
        arguments=a.args[1:] if a.args[:1]==['--'] else a.args
        if not arguments: p.error('需要observerctl参数')
        # Keep the manual lifecycle tool available after rolling back to an old
        # release whose CLI would otherwise start services during activation.
        entry='/opt/robot/local/duck-observer/'+('releases/'+a.tool_version if a.tool_version else 'current')+'/observerctl'
        remote=shlex.join(['python3',entry,*arguments])
        code=run(['ssh',*options,a.target,remote],a.audit_root,'通过审计运行observerctl '+shlex.join(arguments),'持续观测不受命令等待时限影响；控制器不在操作白名单')
    else:
        local=a.audit_root.parent/'board-audit';local.mkdir(exist_ok=True)
        code=run(['scp','-r',*options,a.target+':/var/lib/duck-observer/audit',str(local)],a.audit_root,'同步板端完整门户维护审计','保留原始记录；不要公开私有数据；摘要与证据分开')
    raise SystemExit(code)

if __name__=='__main__':main()
