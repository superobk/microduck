#!/usr/bin/env python3
"""Real Rust ONNX inference + simulated joints, isolated from all robot services.

allow_motion=true is used ONLY with --fake, an isolated socket and runtime. This
exercises the control thread's actual inference-error latch, not a physics test
or permission to change a deployed morphology. Original policy bytes are hashed
before and after. The invalid-state file is an upstream test fixture.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import onnx
from onnx import helper as h, TensorProto as T

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def rpc(path,method,params=None):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.settimeout(2);s.connect(str(path));s.sendall(json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params or {}}).encode()+b'\n')
        return json.loads(s.makefile('rb').readline())['result']
def trial(binary,model,library,out,expect_fault,expect_load_error=False):
    before=sha(model);out.mkdir(parents=True,exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='md-onnx-',dir='/tmp') as tmp:
        tmp=Path(tmp);sock=tmp/'s';profile=tmp/'m.json';config=tmp/'robotd.toml'
        profile.write_text(json.dumps({'schema_version':1,'profile':'headless','mouth_present':True,'allow_motion':True}))
        config.write_text('[policy]\nwalk='+json.dumps(str(model.resolve()))+'\nstand="none"\nsitstand="none"\nground_pick="none"\nkick_left="none"\nkick_right="none"\nroulade="none"\n[audio]\nenabled=false\n[safety]\nbattery_empty_shutdown=false\n')
        env=dict(os.environ,MICRODUCK_MORPHOLOGY=str(profile),DUCK_RUNTIME_DIR=str(tmp/'r'),ORT_DYLIB_PATH=str(library.resolve()))
        with (out/'robotd.log').open('wb') as log:
            p=subprocess.Popen([str(binary.resolve()),'--fake','--params',str(config),'--socket',str(sock)],env=env,stdout=log,stderr=log)
            try:
                until=time.monotonic()+15
                while not sock.exists():
                    assert p.poll() is None,'fake exited'
                    assert time.monotonic()<until,'socket timeout';time.sleep(.05)
                # Socket binding precedes the first published sensor tick.
                # Readiness is an explicit condition, not a guessed sleep.
                while not rpc(sock,'robot.morphology')['imu_ready']:
                    assert time.monotonic()<until,'IMU readiness timeout';time.sleep(.05)
                if expect_load_error:
                    enabled=rpc(sock,'robot.enable',{'on':True});assert not enabled['accepted'],enabled
                    assert not rpc(sock,'robot.init')['accepted']
                    report=rpc(sock,'robot.morphology');health=rpc(sock,'robot.health')
                    assert not report['policy_enabled'] and not health['healthy']
                    result={'model':model.name,'sha256':before,'model_unchanged':sha(model)==before,'expect_load_error':True,'pass':True,'fake_only':True,'morphology':report,'health':health}
                    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True);return
                enabled=rpc(sock,'robot.enable',{'on':True})
                assert enabled['accepted'],enabled
                time.sleep(5)
                report=rpc(sock,'robot.morphology');health=rpc(sock,'robot.health')
                if expect_fault:
                    assert report['fault_latched'] and report['torque_off_acknowledged'],report
                    assert 'inference fault' in report['fault_reason'],report
                    assert not health['healthy'] and 'inference fault' in health['reason'],health
                    assert not report['policy_enabled']
                    assert not rpc(sock,'robot.init')['accepted']
                    assert not rpc(sock,'robot.enable',{'on':True})['accepted']
                    assert rpc(sock,'robot.morphology.rearm')['accepted']
                    until=time.monotonic()+3
                    while rpc(sock,'robot.morphology')['rearm_result']!='cleared':
                        assert time.monotonic()<until,'rearm not cleared';time.sleep(.05)
                    cleared=rpc(sock,'robot.morphology');assert not cleared['policy_enabled']
                else:
                    assert not report['fault_latched'] and report['policy_enabled'],report
                    assert health['healthy'],health
                assert sha(model)==before,'policy file changed'
                result={'model':model.name,'sha256':before,'model_unchanged':True,'expect_inference_fault':expect_fault,'pass':True,'fake_only':True,'morphology':report,'health':health}
                (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
            finally:
                p.terminate()
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['robotd','walk','invalid-state','ort-library','out']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();trial(a.robotd,a.walk,a.ort_library,a.out/'real-reference',False)
    trial(a.robotd,a.invalid_state,a.ort_library,a.out/'invalid-state-load-rejected',False,True)
    # TEST FIXTURE ONLY, created in audit outputs. Sqrt(0) passes warm-up;
    # Sqrt(gravity_z=-1) fails actual runtime inference with 14 NaN actions.
    # No trained/reference ONNX is edited or replaced.
    fixture=a.out/'runtime-nan-fixture.onnx'
    nodes=[h.make_node('Gather',['obs','index'],['gz'],axis=1),h.make_node('Sqrt',['gz'],['bad']),h.make_node('Expand',['bad','shape'],['actions'])]
    graph=h.make_graph(nodes,'fault-fixture',[h.make_tensor_value_info('obs',T.FLOAT,[1,61])],[h.make_tensor_value_info('actions',T.FLOAT,[1,14])],
                       [h.make_tensor('index',T.INT64,[1],[5]),h.make_tensor('shape',T.INT64,[2],[1,14])])
    model=h.make_model(graph,opset_imports=[h.make_opsetid('',13)]);model.ir_version=8;onnx.checker.check_model(model);onnx.save(model,fixture)
    trial(a.robotd,fixture,a.ort_library,a.out/'runtime-inference-fault',True)
if __name__=='__main__':main()
