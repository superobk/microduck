#!/usr/bin/env python3
"""Generate paired Mermaid source / readable SVG from one graph definition.

SVG is a deterministic code-native rendering (not a screenshot of Mermaid).
Nodes, edges and labels share the same source; no browser/network is required.
"""
from pathlib import Path
from html import escape

GRAPHS={
'sensor-model-dataflow':('传感器到原模型的数据流',[
('bus','IMU200 + 11真实舵机\n读取、有限值与新鲜度检查',0,0),('fixed','缺失头颈30–33\n固定HOME，速度/电流0',0,1),
('obs','15逻辑槽 → 跳过嘴 → 61维观测\nHOME仅减一次',1,0),('model','原ONNX（文件不变）\n61输入 → 14输出',2,0),
('action','完整14项模型动作',3,0),('history','原始14项 → 下帧历史\n包含未执行的头颈动作',3,1),
('target','散射15槽 → 原缩放/滤波\n头颈固定目标',4,0),('pack','仅打包真实ID\n右腿动作9–13',5,0),('out','11真实设备写入\n嘴独立命令，故障门禁',6,0)],
[('bus','obs','按ID回填'),('fixed','obs','仅虚拟缺失槽'),('obs','model','f32'),('model','action','有限值'),('action','history','完整保留'),('history','obs','下一帧'),('action','target','跳过嘴'),('target','pack','投影'),('pack','out','物理集合')]),
'id-slot-mapping':('设备、逻辑槽与动作索引',[
('left','左腿 ID20–24\n逻辑槽0–4 / 动作0–4',0,0),('right','右腿 ID10–14\n逻辑槽10–14 / 动作9–13',0,1),
('slots','15逻辑槽保持原顺序\n缺失头颈槽5–8不删除',1,0),('head','头颈 ID30–33\n虚拟HOME / 不在物理总线',1,1),
('policy','原策略14动作\n嘴槽9不在策略动作中',2,0),('mouth','嘴 ID34 / 逻辑槽9\n独立执行器',2,1),
('physical','物理打包：20–24,34,10–14\n合并读取前加IMU200',3,0)],
[('left','slots','回填'),('right','slots','不前移'),('head','slots','投影'),('slots','policy','排除嘴'),('policy','physical','scatter/pack'),('mouth','physical','独立嘴命令')]),
'fault-rearm-state':('故障锁止与显式复位',[
('ready','已就绪 / 受门禁显式启用',0,0),('fault','首个故障：跌倒 / 总线 / 推理\n无效感知 / 可选重复IMU',1,0),
('disable','锁止：策略OFF，丢弃排队init\nhealth报告故障',2,0),('off','关闭真实设备扭矩并逐ID回读',3,0),
('retry','失败：未证明已OFF\n支撑/物理切电，继续重试',3,1),('queued','成功OFF后等待rearm请求\naccepted仅表示排队',4,0),
('judge','控制线程以本tick新鲜样本判定\n直立/静止/无错/策略OFF',5,0),('refused','未满足：rearm_result=refused\n锁止继续',5,1),
('cleared','cleared 且 fault_latched=false\n动力/策略仍OFF',6,0),('init','另一次显式daemon init/enable\nCLI独立init被拒绝',7,0)],
[('ready','fault','异常'),('fault','disable','同tick'),('disable','off','OFF请求'),('off','retry','失败'),('retry','off','重试'),('off','queued','全部回读OFF'),('queued','judge','显式请求'),('judge','refused','条件不足'),('refused','queued','仍待人工请求'),('judge','cleared','条件满足'),('cleared','init','人工确认'),('init','ready','门禁再次检查')]),
'staged-acceptance':('分阶段验收（动力阶段暂缓）',[
('software','软件/自审/离线验证\n本轮候选交付',0,0),('review','人的独立review与SSH身份核对\n当前仍有未验证项',1,0),
('fake','备份 → 独立候选安装 → fake\n所有硬件服务停止',2,0),('supply','供电3.7–6.0V合规实测\n机械/限位/IMU/质量/急停',3,0),
('sense','人在场、支撑、通信与感知\n普通/Fast分别验收',4,0),('home','支撑下HOME逐关节验收',5,0),
('stand','零速60秒 × 5次\n无跌倒/锁止/滑脚/顶限位',6,0),('walk','0.03m/s、0.10rad/s起步\n各工况5次；标尺/录像测量',7,0),
('stop','任一步异常：停止/支撑/切电\n保留失败轨迹',4,1)],
[('software','review','候选'),('review','fake','review通过/SSH恢复'),('fake','supply','本轮到此，后续另审'),('supply','sense','前置全部验收'),('sense','home','通过'),('home','stand','通过'),('stand','walk','通过'),('sense','stop','异常'),('stand','stop','异常'),('walk','stop','异常')]),
'deployment-rollback':('迁移与回退流程',[
('pre','审计只读预检\n身份/实际路径/模型/Runtime',0,0),('ssh','SSH关闭或身份不明\n停止，保留失败；不猜目标',0,1),
('backup','独立RUN目录 + 备份\n记录原服务状态/文件哈希',1,0),('install','校验SCP + 安装独立候选\n官方程序不覆盖；安全hold',2,0),
('fake','隔离fake验收\n候选硬件服务持续STOPPED',3,0),('rollback','只移除本次候选unit\n哈希漂移先停止审查',4,0),
('missing','仍缺头颈：保留本次安全hold\n明确部分回退，不重启',5,0),('full','全部15机械已恢复且人在场确认\n可移除本次hold，仍不自动启动',5,1)],
[('pre','ssh','失败'),('pre','backup','核对/供电隔离'),('backup','install','review通过'),('install','fake','退出0'),('fake','rollback','需回退'),('rollback','missing','11设备'),('rollback','full','full15-restored')]),
'audit-evidence':('每次远端操作的可复用审计',[
('intent','操作意义 + 后续注意事项\n目标/精确脚本/产物哈希',0,0),('before','执行前持久保存operation.json\n脚本按内容SHA归档',1,0),
('run','审计SSH / SCP\n启用host-key校验，凭证不入日志',2,0),('output','流式保存完整输出\n失败/断线也保留',3,0),
('after','执行后退出码/UTC/输出SHA\nsuccess 或 failed_or_interrupted',4,0),('index','日志索引 + 文档证据引用\n不把软件试验写成实机通过',5,0)],
[('intent','before','归档'),('before','run','落盘后执行'),('run','output','完整输出'),('output','after','完成或失败'),('after','index','复用/追踪')])}

def render(directory):
    directory.mkdir(parents=True,exist_ok=True)
    entries=[]
    for name,(title,nodes,edges) in GRAPHS.items():
        height=100+(max(n[2] for n in nodes)+1)*130;positions={n[0]:(85+n[3]*520,55+n[2]*130) for n in nodes}
        lines=['flowchart TD']+[f'  {n[0]}["{n[1].replace(chr(10),"<br/>")}"]' for n in nodes]
        lines += [f'  {a} -->|"{label}"| {b}' for a,b,label in edges]
        (directory/(name+'.mmd')).write_text('\n'.join(lines)+'\n')
        svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="{height}" viewBox="0 0 1120 {height}"><title>{escape(title)}</title>',
             '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10z" fill="#356383"/></marker></defs>',
             '<rect width="100%" height="100%" fill="#f7fafc"/>',f'<text x="85" y="30" font-family="sans-serif" font-size="23" font-weight="bold" fill="#18354b">{escape(title)}</text>']
        for a,b,label in edges:
            ax,ay=positions[a];bx,by=positions[b]
            if ay==by:
                x1=ax+430 if bx>ax else ax;x2=bx if bx>ax else bx+430;y1=ay+(28 if bx>ax else 58);y2=y1
                path=f'M{x1},{y1} L{x2},{y2}';lx=(x1+x2)/2;ly=y1-10
            elif by>ay:
                x1=ax+215;y1=ay+78;x2=bx+215;y2=by;mid=(y1+y2)/2
                path=f'M{x1},{y1} L{x1},{mid} L{x2},{mid} L{x2},{y2}';lx=(x1+x2)/2+7;ly=mid-6
            else:
                x1=ax+430;y1=ay+50;x2=bx+430;y2=by+20;lane=1090
                path=f'M{x1},{y1} L{lane},{y1} L{lane},{y2} L{x2},{y2}';lx=lane-4;ly=(y1+y2)/2
            svg.append(f'<path d="{path}" fill="none" stroke="#356383" stroke-width="2" marker-end="url(#arrow)"/>')
            # Labels sit beside edges; long reverse loops label horizontally at top.
            if by<ay:lx=(x2+1090)/2;ly=y2-7
            svg.append(f'<text x="{lx}" y="{ly}" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#356383">{escape(label)}</text>')
        for ident,label,row,col in nodes:
            x,y=positions[ident];svg.append(f'<rect x="{x}" y="{y}" width="430" height="78" rx="12" fill="{"#e8f3fc" if col==0 else "#fff3de"}" stroke="#5681a0"/>')
            for j,line in enumerate(label.splitlines()):svg.append(f'<text x="{x+215}" y="{y+30+j*24}" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#18354b">{escape(line)}</text>')
        svg.append('</svg>');(directory/(name+'.svg')).write_text('\n'.join(svg)+'\n')
        entries.append(f'## {title}\n\n![{title}]({name}.svg)\n\n[Mermaid源码]({name}.mmd) · [SVG]({name}.svg)\n')
    (directory/'README.md').write_text('# 流程图册\n\nMermaid和SVG由同一节点/边定义生成。SVG使用确定性本地排版，无需联网或运行浏览器。\n\n'+'\n'.join(entries))
if __name__=='__main__':render(Path(__file__).resolve().parents[2]/'docs/zero3w-11servo/diagrams')
