#!/usr/bin/env python3
"""Unchanged ONNX + fixed-head MuJoCo experiments; never connects to a robot.

This is a reference-model feasibility screen, not a hardware certificate. It
uses the original 61/14 ABI, raw last_action, HOME, 0.9 walking scale and 0.7 leg
filter. Mesh/inertia provenance and missing mouth physics are reported explicitly.
Sit/rise are OFFLINE experiments: posture vx=1/0 must not be velocity-clamped,
and their results do not enable the production daemon's refused skill routes.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import onnxruntime as ort
import mujoco as mj

IDS = [20,21,22,23,24,30,31,32,33,34,10,11,12,13,14]
NAMES = ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle',
         'neck_pitch','head_pitch','head_yaw','head_roll','mouth',
         'right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle']
HOME = np.array([0,-.0873,-.4579,-.0049,.4530,.3491,.3491,0,0,0,0,.0873,.4579,.0049,-.4530])
POLICY_SLOTS = [j for j in range(15) if j != 9]
HEAD = [5,6,7,8]
LEGS = [0,1,2,3,4,10,11,12,13,14]
CASES = ['stand','forward','backward','left','right','yaw_left','yaw_right','sit','rise',
         'parking','direction_change','deadman']
COMMANDS = {'forward':[.03,0,0], 'backward':[-.03,0,0], 'left':[0,.03,0],
            'right':[0,-.03,0], 'yaw_left':[0,0,.10], 'yaw_right':[0,0,-.10]}

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path, value): Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def session(path):
    opts = ort.SessionOptions(); opts.intra_op_num_threads = 1; opts.inter_op_num_threads = 1
    s = ort.InferenceSession(str(path), sess_options=opts, providers=['CPUExecutionProvider'])
    ins, outs = s.get_inputs(), s.get_outputs()
    if len(ins)!=1 or len(outs)!=1 or ins[0].shape!=[1,61] or outs[0].shape!=[1,14] or ins[0].type!='tensor(float)':
        raise ValueError(f'{path}: this offline tool requires the alpha feed-forward 61/14 contract')
    return s

def infer(s, obs):
    a = s.run(None,{s.get_inputs()[0].name:np.asarray(obs,dtype=np.float32)[None,:]})[0]
    if a.shape!=(1,14) or not np.isfinite(a).all(): raise ValueError('non-finite/wrong-shape action')
    return a[0].copy()

def contracts(policy_dir):
    results=[]
    for p in sorted(policy_dir.glob('*.onnx')):
        before=sha(p); row={'file':p.name,'sha256':before,'source':'local_reference_not_confirmed_board_policy'}
        try:
            s=session(p); obs=np.zeros(61,np.float32);obs[5]=-1
            first=infer(s,obs); durations=[]
            for _ in range(1000):
                started=time.perf_counter(); a=infer(s,obs);durations.append((time.perf_counter()-started)*1000)
                obs[34:48]=a  # Keep all four unexecuted head outputs in raw history.
            obs[:]=0;obs[5]=-1
            reset=infer(s,obs)
            row.update(interface_pass=True,finite_steps=1000,reset_equal=bool(np.array_equal(first,reset)),
                       latency_ms={k:float(v) for k,v in zip(['p50','p95','p99','max'],
                       [*np.percentile(durations,[50,95,99]),max(durations)])},
                       input_shape=[1,61],output_shape=[1,14])
        except Exception as e: row.update(interface_pass=False,error=str(e))
        row['file_unchanged']=sha(p)==before
        if not row['file_unchanged']: raise RuntimeError('ONNX file changed')
        results.append(row)
    return results

def observation(q, dq, gyro, gravity, previous, twist, locked):
    q=q.copy();dq=dq.copy()
    if locked: q[HEAD]=HOME[HEAD];dq[HEAD]=0
    cmd=np.zeros(13);cmd[:3]=twist
    # HOME head locks make locked-HOME=0. Other fixed angles must use their offset.
    return np.concatenate([gyro,gravity,(q-HOME)[POLICY_SLOTS],dq[POLICY_SLOTS],previous,cmd]).astype(np.float32)

def targets(action, previous, locked, scale):
    scatter=np.zeros(15);scatter[POLICY_SLOTS]=action
    q=HOME+scale*scatter
    if previous is not None:
        alpha=np.full(15,.7);alpha[HEAD]=.5;q=previous+alpha*(q-previous)
    if locked:q[HEAD]=HOME[HEAD]
    return q,scatter

def make_model(path, meshes, locked):
    root=ET.parse(path).getroot()
    compiler=root.find('compiler');compiler.set('meshdir',str(meshes.resolve()))
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    option.set('timestep','0.005')
    # Preserve all source collision shapes/inertias. Remove only visual geoms.
    for parent in root.iter():
        for child in list(parent):
            if child.tag=='geom' and child.get('class')=='visual':parent.remove(child)
    if locked:
        for body in root.iter('body'):
            for joint in list(body):
                if joint.tag=='joint' and joint.get('name') in [NAMES[j] for j in HEAD]:
                    j=NAMES.index(joint.get('name'));angle=HOME[j]
                    axis=np.fromstring(joint.get('axis','0 0 1'),sep=' ');axis/=np.linalg.norm(axis)
                    old=np.fromstring(body.get('quat','1 0 0 0'),sep=' ')
                    rotation=np.r_[math.cos(angle/2),axis*math.sin(angle/2)]; result=np.empty(4)
                    mj.mju_mulQuat(result,old,rotation);body.set('quat',' '.join(map(str,result)))
                    # Removing the hinge gives a truly rigid lock, not a hidden
                    # high-gain head motor that would help the policy balance.
                    body.remove(joint)
        actuator=root.find('actuator')
        for child in list(actuator):
            if child.get('joint') in [NAMES[j] for j in HEAD]:actuator.remove(child)
    world=root.find('worldbody')
    ET.SubElement(world,'geom',name='validation_floor',type='plane',size='5 5 .1',friction='.8 .005 .0001')
    xml=ET.tostring(root,encoding='unicode')
    model=mj.MjModel.from_xml_string(xml)
    return model,xml

def schedule(case,t):
    flag=None
    if case=='sit':flag=t>=2
    if case=='rise':flag=2<=t<6 if t<10 else None
    if flag is not None:return np.array([float(flag),0,0]),flag
    command=np.array(COMMANDS.get(case,[0,0,0]),float) if t>=2 else np.zeros(3)
    if case=='parking':command=np.array([.03,0,0]) if 2<=t<7 else np.zeros(3)
    if case=='direction_change':command=np.array([.03 if t<7 else -.03,0,0]) if 2<=t<12 else np.zeros(3)
    if case=='deadman':
        # Last velocity publication at 7s; upstream deadman is 500ms, zero
        # velocity rather than dropping torque. This is an intent-age experiment.
        command=np.array([.03,0,0]) if 2<=t<=7.5 else np.zeros(3)
    return command,None

def episode(model,walk,sitstand,case,seed,duration,locked,out):
    data=mj.MjData(model);rng=np.random.default_rng(seed)
    trunk=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'trunk_base')
    qadr={};vadr={};act={}
    for j,name in enumerate(NAMES):
        k=mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,name)
        if k>=0:qadr[j]=int(model.jnt_qposadr[k]);vadr[j]=int(model.jnt_dofadr[k])
        k=mj.mj_name2id(model,mj.mjtObj.mjOBJ_ACTUATOR,name)
        if k>=0:act[j]=k
    data.qpos[:7]=[0,0,.12,1,0,0,0]
    for j,a in qadr.items():data.qpos[a]=HOME[j]+(rng.normal(0,.001) if j in LEGS else 0)
    mj.mj_forward(model,data)
    gyro_id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_SENSOR,'angular-velocity')
    gyro_adr=int(model.sensor_adr[gyro_id])
    previous=np.zeros(14,np.float32);previous_target=None;fall_s=0;terminal=None;rows=[];golden=[]
    path=out/f'{"locked" if locked else "full15"}-{case}-seed{seed}.jsonl'
    start_xy=data.xpos[trunk,:2].copy()
    start_yaw=0.0
    with path.open('x') as log:
        for i in range(round(duration*50)):
            t=i/50;twist,flag=schedule(case,t);using_sit=flag is not None
            if using_sit and sitstand is None:terminal='sitstand_model_missing';break
            q=HOME.copy();dq=np.zeros(15)
            for j,a in qadr.items():q[j]=data.qpos[a];dq[j]=data.qvel[vadr[j]]
            R=data.xmat[trunk].reshape(3,3);gravity=R.T@np.array([0,0,-1]);gyro=data.sensordata[gyro_adr:gyro_adr+3].copy()
            obs=observation(q,dq,gyro,gravity,previous,twist,locked)
            if not np.isfinite(obs).all():terminal='non_finite_observation';break
            try:a=infer(sitstand if using_sit else walk,obs)
            except ValueError as e:terminal=str(e);break
            scale=1.0 if using_sit else .9;target,scatter=targets(a,previous_target,locked,scale)
            if i%50==0 and len(golden)<20:
                golden.append(dict(positions=q.tolist(),velocities=dq.tolist(),gyro=gyro.tolist(),gravity=gravity.tolist(),
                    previous_action=previous.tolist(),action=a.tolist(),twist=twist.tolist(),standing=using_sit,
                    posture_flag=flag,previous_target=None if previous_target is None else previous_target.tolist(),
                    expected={'observation':obs.tolist(),'scatter':scatter.tolist(),'targets':target.tolist()}))
            previous=a;previous_target=target
            # Prototype actuator fit, retained from source. It is NOT calibrated
            # to this user's overvolted hardware or a future regulated supply.
            for j,k in act.items():data.ctrl[k]=float(np.clip(target[j],-math.pi,math.pi))
            for _ in range(4):mj.mj_step(model,data)
            qroot=data.xquat[trunk];yaw=math.atan2(2*(qroot[0]*qroot[3]+qroot[1]*qroot[2]),1-2*(qroot[2]**2+qroot[3]**2))
            tilt=math.degrees(math.acos(np.clip(-gravity[2],-1,1)))
            row={'t':t,'trunk_xyz_m':data.xpos[trunk].tolist(),'yaw_rad':yaw,'tilt_deg':tilt,
                 'command':twist.tolist(),'posture_flag':flag,'policy':'sitstand' if using_sit else 'walk',
                 'observation':obs.tolist(),'raw_action':a.tolist(),'targets':target.tolist()}
            log.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
            fall_s=fall_s+.02 if gravity[2]>-.5 else 0
            if fall_s>=.2:terminal='fall_debounce';break
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():terminal='non_finite_physics';break
    if not rows:return {'case':case,'seed':seed,'locked':locked,'terminal':terminal,'simulation_pass':False},golden
    end=rows[-1];delta=np.array(end['trunk_xyz_m'][:2])-start_xy
    passed=terminal is None
    # A nonzero requested command is not measured motion. Require correct sign.
    if case in ('forward','backward'):passed &= delta[0]*(1 if case=='forward' else -1)>.01
    if case in ('left','right'):passed &= delta[1]*(1 if case=='left' else -1)>.01
    if case in ('yaw_left','yaw_right'):passed &= (end['yaw_rad']-start_yaw)*(1 if case=='yaw_left' else -1)>.05
    if case=='sit':passed &= end['trunk_xyz_m'][2] < rows[min(99,len(rows)-1)]['trunk_xyz_m'][2]-.015
    if case=='rise':
        at6=rows[min(299,len(rows)-1)]['trunk_xyz_m'][2]
        passed &= end['trunk_xyz_m'][2]>at6+.015 and end['tilt_deg']<20
    if case in ('parking','deadman') and len(rows)>250:
        d=np.linalg.norm(np.array(end['trunk_xyz_m'][:2])-np.array(rows[-251]['trunk_xyz_m'][:2]))/5
        # A stationary robot cannot certify stopping after successful walking.
        moved=rows[min(349,len(rows)-1)]['trunk_xyz_m'][0]-rows[min(99,len(rows)-1)]['trunk_xyz_m'][0]
        passed &= d<.01 and moved>.01
    if case=='direction_change':
        x2=rows[min(99,len(rows)-1)]['trunk_xyz_m'][0]
        x7=rows[min(349,len(rows)-1)]['trunk_xyz_m'][0]
        x12=rows[min(599,len(rows)-1)]['trunk_xyz_m'][0]
        passed &= x7-x2>.01 and x12-x7<-.01
    analyzed=[r['tilt_deg'] for r in rows if r['t']>=5]
    return {'case':case,'seed':seed,'locked':locked,'steps':len(rows),'duration_s':len(rows)/50,
        'terminal':terminal,'simulation_pass':bool(passed),'measured_delta_xy_m':delta.tolist(),
        'measured_final_yaw_rad':end['yaw_rad'],'final_height_m':end['trunk_xyz_m'][2],
        'tilt_p95_deg':float(np.percentile(analyzed,95)) if analyzed else None,'trace':path.name},golden

def compare_golden(executable,profile,frames):
    env=dict(os.environ,MICRODUCK_MORPHOLOGY=str(profile.resolve()))
    p=subprocess.run([str(executable.resolve())],input=''.join(json.dumps({k:v for k,v in f.items() if k!='expected'})+'\n' for f in frames),
                     capture_output=True,text=True,env=env,check=True)
    actual=[json.loads(line) for line in p.stdout.splitlines()]
    if len(actual)!=len(frames):raise ValueError('golden frame count differs')
    maximum=0.0
    for f,a in zip(frames,actual):
        for key in ('observation','scatter','targets'):
            delta=float(np.max(np.abs(np.array(f['expected'][key])-a[key])));maximum=max(maximum,delta)
            if delta>2e-6:raise ValueError(f'Rust/Python {key} mismatch {delta}')
    return {'frames':len(frames),'maximum_abs_delta':maximum,'pass':True,'tolerance':2e-6}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--meshes',type=Path,required=True)
    p.add_argument('--policy-dir',type=Path,required=True);p.add_argument('--walk',type=Path,required=True)
    p.add_argument('--sitstand',type=Path);p.add_argument('--golden',type=Path,required=True)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--duration',type=float,default=30);p.add_argument('--seeds',type=int,nargs='+',default=[0,1,2])
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    c=contracts(args.policy_dir);write(args.out/'contracts.json',c)
    walk=session(args.walk);sit=session(args.sitstand) if args.sitstand else None
    locked,xml=make_model(args.model,args.meshes,True);full,_=make_model(args.model,args.meshes,False)
    (args.out/'locked-model.xml').write_text(xml)
    provenance={'source_model':str(args.model.resolve()),'source_model_sha256':sha(args.model),
        'mesh_hashes':{x.name:sha(x) for x in sorted(args.meshes.glob('*.stl'))},
        'walk_sha256':sha(args.walk),'sitstand_sha256':sha(args.sitstand) if args.sitstand else None,
        'physics_timestep_s':.005,'control_hz':50,'total_mass_kg':float(locked.body_mass.sum()),
        'locked_head_rad':HOME[HEAD].tolist(),'head_actuators_removed':True,
        'mouth_physics_present':False,'leg_actuators':locked.nu,
        'assumptions':['Source mesh/inertias are retained, including mass of originally fitted head servos.',
                       'Physical removed-servo mass, braces, battery position and supply response are unmeasured.',
                       'Offline sit/rise bypass the production skill prohibition only in this isolated harness.',
                       'All policies are local reference files; board identity remains unverified.'],
        'hardware_verified':False}
    write(args.out/'provenance.json',provenance)
    results=[];golden=[]
    for case in CASES:
        for seed in args.seeds:
            r,g=episode(locked,walk,sit,case,seed,args.duration,True,args.out);results.append(r);golden.extend(g[:3])
            print(json.dumps(r),flush=True)
    for case in ('stand','forward'):
        for seed in args.seeds:
            r,_=episode(full,walk,sit,case,seed,args.duration,False,args.out);results.append(r)
            print(json.dumps(r),flush=True)
    write(args.out/'golden-inputs.json',golden)
    gold=compare_golden(args.golden,args.profile,golden);write(args.out/'golden-comparison.json',gold)
    basic=CASES[:9]
    passed=[k for k in basic if all(r['simulation_pass'] for r in results if r['locked'] and r['case']==k)]
    summary={'report_kind':'reference_simulation_not_hardware_certificate','interface_pass_count':sum(x['interface_pass'] for x in c),
             'interface_total':len(c),'basic_scenarios':basic,'simulation_passed_scenarios':passed,
             'simulation_pass_count':len(passed),'hardware_pass_count':0,'golden':gold,'episodes':results,'provenance':provenance}
    write(args.out/'summary.json',summary)
    return 0

if __name__=='__main__':raise SystemExit(main())
