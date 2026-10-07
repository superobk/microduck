#!/usr/bin/env python3
"""Append readable, deduplicated board-operation evidence after audited SCP.

This runs locally and never executes the reconstructed commands. Keep exact IPC,
exit codes and evidence links separate from equivalent CLI examples; successful
playback or queued actions must not be rewritten as completed hardware acceptance.
Full stdout/stderr remain in the original evidence, rather than being republished.
"""
import argparse
import json
from pathlib import Path
import shlex

MEANING={
    'observe':'切换门户采集；开启时复核UART所有权和真实ID，关闭时释放自有串口',
    'record':'切换原始遥测记录；实时观测不受记录容量影响',
    'calibrate':'只修改门户显示校正，原始姿态和控制器配置保留',
    'service':'只切换指定ToF/媒体服务；依赖保护和新帧检查必须通过',
    'restore':'按事务恢复服务；发生人工状态或配置漂移时拒绝覆盖',
    'audio':'播放门户短音；数字幅度受限，不修改持久混音器',
    'audio_stop':'停止门户自有播放器，不结束其他音频进程',
    'audio_heard':'保存现场听音反馈；必须来自实际人工确认',
    'export':'导出当前状态、审计、校正和统计证据',
    'fast_test':'独立Fast诊断；不更改普通读取模式，也不写入控制寄存器'}

def equivalent(action,args):
    if action in ('observe','record'):return [action,'on' if args.get('enabled') else 'off']
    if action=='service':return ['service',args['unit'],args['verb']]
    if action=='restore':return ['restore','--transaction',args['transaction']]
    if action=='calibrate':return ['calibrate',args['mode']]+(['--mount',*map(str,args['mount'])] if 'mount' in args else [])
    if action=='audio':return ['audio','test','--level',str(args.get('level',.1))]
    if action=='audio_stop':return ['audio','stop']
    if action=='audio_heard':return ['audio','heard' if args.get('heard') else 'not-heard']
    if action=='fast_test':return ['fast-test']
    if action=='export':return ['export']
    return None

def append(source,destination):
    destination.parent.mkdir(parents=True,exist_ok=True)
    prior=destination.read_text() if destination.exists() else ''
    entries=[]
    for path in sorted(source.glob('events-*.jsonl')):
        with path.open() as stream:
            for line_number,line in enumerate(stream,1):
                row=json.loads(line);ident=row['id'];kind=row['kind']
                if '<!-- audit:'+ident+' -->' in prior:continue
                if kind not in {'operation_queued','operation_finished','operation_failed','command_finished',
                    'audio_player_started','audio_completed','release_started','release_completed','install_completed'}:continue
                action=row.get('action');reason=MEANING.get(action,'保留自有安装、版本切换或播放器执行证据')
                text=[f'\n<!-- audit:{ident} -->\n### {row["utc"]} · {action or kind}\n',
                    f'操作意义：{reason}。证据：[原始审计第{line_number}行]({path.resolve()}:{line_number})。',
                    f'记录类型：`{kind}`；事务：`{row.get("transaction","未提供")}`。']
                if kind=='operation_queued':
                    text+=['已提交的实际本地IPC（排队不等于完成）：',
                        '```json\n'+json.dumps({'action':action,'params':row.get('args',{})},ensure_ascii=False)+'\n```']
                    cli=equivalent(action,row.get('args',{}))
                    if cli:text+=['等价复现参数（须经使用指南中的SSH审计包装器，不自动执行）：',
                        '```bash\nobserverctl '+shlex.join(cli)+'\n```']
                if kind in {'command_finished','audio_player_started'}:
                    text+=['实际执行的程序参数：','```bash\n'+shlex.join(row['argv'])+'\n```']
                if 'exit_code' in row:text+=['退出码：`'+str(row['exit_code'])+'`。']
                if kind=='operation_finished':
                    text+=['完成结果：','```json\n'+json.dumps(row.get('result'),ensure_ascii=False,indent=2)+'\n```']
                if kind=='operation_failed':text+=['失败原因：'+str(row.get('reason'))+'。']
                recovery={'service':'用原事务ID执行restore；后续人工变化发生时先核对，不强制覆盖',
                    'calibrate':'撤销校正；原始数据继续展示', 'audio':'停止自有播放器；现场听音另行确认',
                    'observe':'关闭门户采集以释放UART，再排查占用/身份/掉线',
                    'record':'停止记录并检查磁盘，保留审计'}
                text+=['预期与停止条件：维护任务须有完成结果，失败或审计故障立即停止对应操作；服务恢复须有新帧。',
                    '恢复方法：'+recovery.get(action,'按使用指南恢复自有版本或对应服务事务，保留完整证据')+'。\n']
                entries.append('\n\n'.join(text))
    if entries:
        with destination.open('a') as stream:
            if not prior:stream.write('# 门户板端操作增量摘要\n\n本文件只从已同步的实际审计追加，按事件ID去重。等价CLI不是新增执行记录。时间为UTC。\n')
            stream.write(''.join(entries))
    return len(entries)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    print(json.dumps({'appended':append(args.source,args.output),'output':str(args.output)},ensure_ascii=False))
