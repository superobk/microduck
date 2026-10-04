#!/usr/bin/env python3
"""Audited SSH transport for remote_candidate.py; never starts physical motion.

Every remote query/write, including failed connections and SCP, is recorded by
audit_command.run before execution. Exact scripts and their hashes are copied
into the local audit run. Host-key checking stays enabled; credentials never
appear in argv. Mutating steps require an operator's physical power-isolation
acknowledgment and an explicitly reviewed preflight, not an elapsed timeout.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import re
from audit_command import run

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['preflight','prepare','backup','install','fake','rollback'])
    p.add_argument('--target',required=True);p.add_argument('--run-id',required=True);p.add_argument('--audit-root',type=Path,required=True)
    p.add_argument('--binary',type=Path);p.add_argument('--source-commit');p.add_argument('--power-isolated',action='store_true')
    p.add_argument('--preflight-reviewed',action='store_true');p.add_argument('--plan-only',action='store_true')
    p.add_argument('--full15-restored',action='store_true',help='operator verified all 15 physical joints restored; rollback may remove owned holds')
    a=p.parse_args()
    # Validate before SCP too: remote validation happens after the upload stage.
    if not re.fullmatch(r'[A-Za-z0-9_+-]+',a.run_id):p.error('unsafe run ID')
    if a.target.startswith('-') or any(c.isspace() for c in a.target):p.error('invalid SSH target')
    if a.action in ('backup','install','rollback') and not (a.power_isolated and a.preflight_reviewed):p.error('operator power isolation and reviewed preflight required')
    if a.action=='install' and (a.binary is None or not a.source_commit):p.error('--binary and --source-commit required')
    if a.action=='install' and not re.fullmatch(r'[0-9a-f]{40}',a.source_commit):p.error('source commit must be the reviewed 40-character Git SHA')
    script=Path(__file__).with_name('remote_candidate.py');data=script.read_bytes();digest=hashlib.sha256(data).hexdigest()
    artifact=a.audit_root/'scripts'/digest;artifact.mkdir(parents=True,exist_ok=True)
    shutil.copy2(script,artifact/script.name)
    args=[a.action,a.run_id]
    if a.power_isolated:args+=['--power-isolated']
    if a.full15_restored:args+=['--full15-restored']
    if a.action=='install':
        bd=hashlib.sha256(a.binary.read_bytes()).hexdigest();args+=['--sha256',bd,'--source-commit',a.source_commit]
        helper=Path(__file__).parents[1]/'headless-v015-r1/tools/prepare_config.py'
        hd=hashlib.sha256(helper.read_bytes()).hexdigest();args+=['--config-helper-sha',hd]
        shutil.copy2(helper,artifact/helper.name)
        destination=f'{a.target}:/opt/robot/local/zero3w-11servo/{a.run_id}/robotd.staged'
        if not a.plan_only:
            (artifact/'binary-transfer.json').write_text(json.dumps({'source':str(a.binary.resolve()),'sha256':bd,'source_commit':a.source_commit,'destination':destination},indent=2))
            code=run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10',str(a.binary.resolve()),destination],a.audit_root,'上传候选二进制到本次独立目录','校验不符立即停止；官方路径不覆盖。')
            if code:return code
            code=run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10',str(helper.resolve()),f'{a.target}:/opt/robot/local/zero3w-11servo/{a.run_id}/prepare_config.py'],a.audit_root,'上传经审查的配置复制工具','生成独立配置，保持原 walk/stand、增益、缩放与滤波。')
            if code:return code
    loader="import base64;exec(compile(base64.b64decode("+repr(base64.b64encode(data).decode())+"),'audited-remote_candidate.py','exec'))"
    remote=shlex.join(['python3','-c',loader,*args])
    argv=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10',a.target,remote]
    if a.plan_only:
        print(json.dumps({'action':a.action,'target':a.target,'script_sha256':digest,'remote_args':args,'remote_executed':False},indent=2));return 0
    return run(argv,a.audit_root,f'Zero3W {a.action}：执行已归档的精确脚本','失败则停止本阶段；所有硬件服务保持停止，禁止跳过供电验收。')
if __name__=='__main__':raise SystemExit(main())
