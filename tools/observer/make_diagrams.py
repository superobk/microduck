#!/usr/bin/env python3
"""Generate matching Mermaid sources and standalone SVGs without external CDN.

Both views share the same ordered node/edge definition. SVG is a simple readable
layout rather than a dependency on a remote Mermaid rendering service.
"""
import argparse
from html import escape
from pathlib import Path

DIAGRAMS={
 'data-flow':('传感器到页面的数据流',['真实舵机 / IMU 200','单一只读 UART 采集器','原 robot.state / tofd','共享快照 / 年龄 / 告警','非 root Web / SSE','3D 姿态 / 热力图 / 状态'],[(0,1),(1,3),(2,3),(3,4),(4,5)]),
 'ownership':('资源所有权与持续观测',['控制器状态 + 实际 FD','已运行：订阅原服务','已停止：手动读 UART','多个页面共享采集器','暂停画面：仅当前页','停止采集：释放 UART'],[(0,1),(0,2),(1,3),(2,3),(3,4),(3,5)]),
 'service-switch':('感知服务事务与恢复',['操作先 fsync 审计','检查依赖 + 停止条件','保存原状态和指纹','串行切换 tofd / mediad','新帧证明恢复成功','事务恢复：漂移则拒绝'],[(0,1),(1,2),(2,3),(3,4),(4,5)]),
 'calibration':('IMU 显示校正与撤销',['原始四元数 / 角速度','安装旋转：默认 +90°Y','新鲜度 / 静止样本检查','偏置 / 相对参考归零','独立配置及历史版本','撤销校正 / 原始重力可见'],[(0,1),(1,2),(2,3),(3,4),(4,5)]),
 'release-rollback':('独立文件与明确手动启动',['预检 / 审查 / 哈希清单','备份并安装 / 无开机链接','切换版本 / 始终保持停止','明确 station start / 健康检查','手动 observe on / 20或50Hz','station stop / 释放自有资源'],[(0,1),(1,2),(2,3),(3,4),(4,5)]),
}

def generate(output):
    output.mkdir(parents=True,exist_ok=True)
    for name,(title,nodes,edges) in DIAGRAMS.items():
        positions=[(30+(i%3)*340,100+(i//3)*165) for i in range(len(nodes))]
        mmd=['flowchart LR']+[f'  n{i}["{text}"]' for i,text in enumerate(nodes)]+[f'  n{a} --> n{b}' for a,b in edges]
        (output/(name+'.mmd')).write_text('\n'.join(mmd)+'\n')
        svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1040" height="440" viewBox="0 0 1040 440" role="img">',
             f'<title>{escape(title)}</title><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6" fill="#7da78f"/></marker></defs>',
             '<rect width="1040" height="440" fill="#faf9f2"/>',f'<text x="30" y="48" font-size="26" font-family="sans-serif" fill="#385347">{escape(title)}</text>']
        for a,b in edges:
            x,y=positions[a];u,v=positions[b]
            if y==v: path=f'M{x+280},{y+36} L{u},{v+36}'
            else:path=f'M{x+140},{y+72} C{x+140},{y+110} {u+140},{v-40} {u+140},{v}'
            svg.append(f'<path d="{path}" stroke="#7da78f" stroke-width="2" fill="none" marker-end="url(#arrow)"/>')
        for i,(text,(x,y)) in enumerate(zip(nodes,positions)):
            svg.extend([f'<rect x="{x}" y="{y}" width="280" height="72" rx="16" fill="#e3eee1" stroke="#9bb79a"/>',f'<text x="{x+140}" y="{y+43}" text-anchor="middle" font-size="18" font-family="sans-serif" fill="#385347">{escape(text)}</text>'])
        svg.append('<text x="30" y="414" font-size="14" font-family="sans-serif" fill="#607268">门户维护不启动控制器 / 不写舵机目标或扭矩 / 审计与回退独立保留</text></svg>')
        (output/(name+'.svg')).write_text('\n'.join(svg)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();generate(a.output)
