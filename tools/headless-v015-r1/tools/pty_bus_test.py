#!/usr/bin/env python3
"""Linux-only PTY integration test against a BUILT native robotd binary.

No hardware is touched. Implements ordinary DXL Protocol 2 ping/read/write,
Sync Read/Write, Fast Sync Read (0x8a running block CRC) and reboot separately.
This checks actual runtime bus membership and scatter/gather; it is NOT an
IMU firmware emulator or a physics/ONNX/balance test.
"""
from __future__ import annotations
import argparse
import errno
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time
import sys

HEADER=b"\xff\xff\xfd\x00"
JOINT_IDS=[20,21,22,23,24,30,31,32,33,34,10,11,12,13,14]
HOME=[0.,-.0873,-.4579,-.0049,.453,.3491,.3491,0.,0.,0.,0.,.0873,.4579,.0049,-.453]


def crc16(data: bytes) -> int:
    crc=0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc=((crc<<1)^0x8005) & 0xffff if crc & 0x8000 else (crc<<1)&0xffff
    return crc


def stuff(data: bytes) -> bytes: return data.replace(b"\xff\xff\xfd",b"\xff\xff\xfd\xfd")
def unstuff(data: bytes) -> bytes: return data.replace(b"\xff\xff\xfd\xfd",b"\xff\xff\xfd")

def packet(device: int, instruction: int, params: bytes=b"") -> bytes:
    body=stuff(bytes([instruction])+params)
    raw=HEADER+bytes([device])+struct.pack("<H",len(body)+2)+body
    return raw+struct.pack("<H",crc16(raw))


class Bus:
    def __init__(self, fd: int, ids: list[int]):
        self.fd=fd; self.ids=set(ids)|{200}; self.dropped=set(); self.events=[]
        self.stop=threading.Event(); self.error=None; self.memory={}; self.count=0
        self.reject_off=False
        for i in self.ids:
            mem=bytearray(256); mem[:3]=struct.pack("<HB",1200,46)
            mem[7]=i; mem[8]=3; mem[9]=0
            if i in JOINT_IDS:
                mem[132:136]=struct.pack("<i",2048+8*JOINT_IDS.index(i))
            mem[144:147]=struct.pack("<HB",50,40)
            self.memory[i]=mem
        self.thread=threading.Thread(target=self.run,daemon=True)
    def start(self): self.thread.start()
    def close(self): self.stop.set(); self.thread.join(timeout=2)
    def send(self,i:int,data:bytes=b"",err:int=0):
        os.write(self.fd,packet(i,0x55,bytes([err])+data))
    def read(self,i:int,address:int,size:int):
        if i not in self.ids or i in self.dropped: return
        if i==200 and address==124 and size==12:
            self.count+=1
            # Vary valid finite half-floats only to avoid a frozen-block diagnostic.
            # Orientation fidelity is intentionally NOT claimed by this test.
            # Wire order is signed i16 gyro counts THEN xyz half quaternion.
            # Sensor is mounted +90deg about Y. Encode that orientation, so the
            # decoder's sensor->trunk transform yields an upright trunk.
            data=struct.pack("<3h3e",1+self.count%10,0,0,0.,math.sqrt(.5),0.)
        else: data=bytes(self.memory[i][address:address+size])
        self.send(i,data)
    def write(self,i:int,address:int,data:bytes,ack:bool):
        if i not in self.ids or i in self.dropped: return
        self.events.append(("write",i,address,list(data)))
        # An acknowledged instruction is not proof of torque OFF: emulate a
        # device that retains its ON register, exercising the real readback.
        if self.reject_off and address==64 and data==b'\x00':
            self.memory[i][64]=1
            if ack:self.send(i)
            return
        self.memory[i][address:address+len(data)]=data
        if ack: self.send(i)
    def handle(self,device:int,instruction:int,p:bytes):
        self.events.append(("instruction",device,instruction,list(p)))
        if instruction==1:
            if device in self.ids and device not in self.dropped: self.send(device,bytes(self.memory[device][:3]))
        elif instruction==2:
            address,size=struct.unpack("<HH",p); self.read(device,address,size)
        elif instruction==3:
            address=struct.unpack("<H",p[:2])[0]
            for i in (sorted(self.ids) if device==254 else [device]): self.write(i,address,p[2:],device!=254)
        elif instruction==8:
            if device in self.ids and device not in self.dropped: self.send(device)
        elif instruction==0x82:
            address,size=struct.unpack("<HH",p[:4]); ids=list(p[4:])
            self.events.append(("sync_read",ids,address,size))
            for i in ids: self.read(i,address,size)
        elif instruction==0x8a:
            address,size=struct.unpack('<HH',p[:4]);ids=list(p[4:])
            self.events.append(('fast_sync_read',ids,address,size))
            if any(i not in self.ids or i in self.dropped for i in ids):return
            # rustypot 1.8 / ROBOTIS Fast Sync status: broadcast ID, no stuffing,
            # ERROR+ID+DATA+CRC per device; each CRC covers the entire prefix.
            raw=HEADER+bytes([254])+struct.pack('<H',1+len(ids)*(size+4))+b'\x55'
            for i in ids:
                if i==200 and address==124 and size==12:
                    self.count+=1;data=struct.pack('<3h3e',1+self.count%10,0,0,0.,math.sqrt(.5),0.)
                else:data=bytes(self.memory[i][address:address+size])
                raw+=bytes([0,i])+data;raw+=struct.pack('<H',crc16(raw))
            os.write(self.fd,raw)
        elif instruction==0x83:
            address,size=struct.unpack("<HH",p[:4]); entries=p[4:]
            if len(entries)%(size+1): raise RuntimeError("malformed sync write")
            ids=[]
            for offset in range(0,len(entries),size+1):
                i=entries[offset]; ids.append(i); self.write(i,address,entries[offset+1:offset+1+size],False)
            self.events.append(("sync_write",ids,address,size))
        else: raise RuntimeError(f"unsupported instruction {instruction:#x}; test uses plain Sync Read")
    def run(self):
        buf=b""
        try:
            while not self.stop.is_set():
                if not select.select([self.fd],[],[],.05)[0]: continue
                try: chunk=os.read(self.fd,65536)
                except OSError as e:
                    if e.errno==errno.EIO: time.sleep(.01); continue
                    raise
                buf+=chunk
                while len(buf)>=7:
                    at=buf.find(HEADER)
                    if at<0: buf=buf[-3:]; break
                    buf=buf[at:]
                    if len(buf)<7: break
                    size=7+struct.unpack("<H",buf[5:7])[0]
                    if len(buf)<size: break
                    raw,buf=buf[:size],buf[size:]
                    if crc16(raw[:-2])!=struct.unpack("<H",raw[-2:])[0]: raise RuntimeError("bad request CRC")
                    body=unstuff(raw[7:-2]); self.handle(raw[4],body[0],body[1:])
        except BaseException as e: self.error=e


def rpc(sock:Path,method:str,params:dict|None=None):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.settimeout(2); s.connect(str(sock)); s.sendall(json.dumps({"jsonrpc":"2.0","id":1,"method":method,"params":params or {}}).encode()+b"\n")
        with s.makefile("rb") as f:
            if method=="robot.subscribe":
                f.readline()
                for _ in range(20):
                    msg=json.loads(f.readline())
                    if msg.get("method")=="robot.state": return msg["params"]
                raise RuntimeError("no state notification")
            msg=json.loads(f.readline())
    if "error" in msg: raise RuntimeError(str(msg["error"]))
    return msg["result"]


def notify(sock,method,params):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.connect(str(sock));s.sendall(json.dumps({'jsonrpc':'2.0','method':method,'params':params}).encode()+b'\n')

def until(sock,predicate,seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        value=rpc(sock,'robot.morphology')
        if predicate(value):return value
        time.sleep(.1)
    raise AssertionError(f'morphology condition timed out: {value}')

def case(binary:Path,mouth:bool,output:Path,fast=False,full=False):
    active=JOINT_IDS[:] if full else [i for i in JOINT_IDS if i not in [30,31,32,33] and (mouth or i!=34)]
    master,slave=pty.openpty(); bus=Bus(master,active); bus.start()
    proc=None
    with tempfile.TemporaryDirectory(prefix="headless-pty-") as tmp:
        t=Path(tmp); sock=t/'robotd.sock'; cfg=t/'robotd.toml'; morph=t/'morphology.json'; log=t/'robotd.log'
        cfg.write_text(f'[bus]\nport = {json.dumps(os.ttyname(slave))}\nfast_sync_read = {str(fast).lower()}\n[policy]\nenabled = false\n[audio]\nenabled = false\n[safety]\nbattery_empty_shutdown = false\n')
        morph.write_text(json.dumps(dict(schema_version=1,profile="full15" if full else "headless",mouth_present=mouth,allow_motion=full)))
        env=dict(os.environ,MICRODUCK_MORPHOLOGY=str(morph),RUST_LOG="warn",DUCK_RUNTIME_DIR=str(t/'runtime'))
        try:
            with log.open('wb') as lf:
                proc=subprocess.Popen([str(binary),'--socket',str(sock),'--params',str(cfg),'--no-policy'],env=env,stdout=lf,stderr=lf)
                deadline=time.monotonic()+20
                while True:
                    if proc.poll() is not None: raise RuntimeError(f'robotd exited: {log.read_text()}')
                    if bus.error: raise RuntimeError(f'bus emulator: {bus.error}')
                    try:
                        st=rpc(sock,'robot.subscribe',{'hz':5})
                        if 'joints' in st: break
                    except (OSError,ValueError,RuntimeError): pass
                    if time.monotonic()>deadline: raise RuntimeError(f'timeout: {log.read_text()}')
                    time.sleep(.1)
                for j,i in enumerate(JOINT_IDS):
                    expected=(2*math.pi*(2048+8*j)/4096-math.pi) if i in active else HOME[j]
                    assert abs(st['joints'][j]-expected)<1e-6,(i,st['joints'][j],expected)
                report=rpc(sock,'robot.morphology'); assert report['active_motor_ids']==active
                if full:
                    assert rpc(sock,'robot.head',dict(neck_pitch=.1,head_pitch=.1,head_yaw=.1,head_roll=.1))['accepted']
                else:
                    verify_headless(sock,bus,active)
                time.sleep(.5)
                sync=[e for e in bus.events if e[0] in ('sync_read','sync_write','fast_sync_read')]
                assert sync,'no actual sync transactions'
                allowed=set(active)|{200}
                for e in sync: assert set(e[1])<=allowed,e
                read_kind='fast_sync_read' if fast else 'sync_read'
                assert any(e[0]==read_kind and e[1]==[200]+active for e in sync),'selected combined IMU/motor protocol not exercised'
                assert any(e[0]=='sync_write' and e[1]==active for e in sync),'no correctly packed position write'
                assert not any(e[0]=='write' and e[2]==64 and e[3]==[1] for e in bus.events),'bench profile enabled torque'
                if not full:verify_fault_reset(sock,bus)
                r=rpc(sock,'robot.morphology')
                output.write_text(json.dumps({'mouth_present':mouth,'full15':full,'fast_sync_read':fast,'active':active,'fault':r,'events':bus.events},indent=2))
                print(f'PASS PTY devices={len(active)} fast={fast}: scatter, pack, gates, fault/off retry/rearm',flush=True)
        finally:
            if proc is not None:
                proc.terminate()
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            bus.close(); os.close(master); os.close(slave)
            output.with_suffix('.robotd.log').write_text(log.read_text() if log.exists() else '')


def verify_headless(sock,bus,active):
                assert not rpc(sock,'robot.head',dict(neck_pitch=.1,head_pitch=.1,head_yaw=.1,head_roll=.1))['accepted']
                assert not rpc(sock,'robot.do',dict(skill='roulade'))['accepted']
                assert not rpc(sock,'robot.enable',dict(on=True))['accepted']
                for method,params in [('robot.head',dict(neck_pitch=.2,head_pitch=.2,head_yaw=.2,head_roll=.2)),('robot.enable',{'on':True}),('robot.init',{}),('robot.do',{'skill':'roulade'}),('robot.mode',{'mode':'roller'})]:
                    notify(sock,method,params)
                assert not rpc(sock,'robot.init',{})['accepted']
                assert not rpc(sock,'robot.rebootMotors',{'ids':[30]})['accepted']
                assert rpc(sock,'robot.mouth',{'open':.5})['accepted']==(34 in active)


def verify_fault_reset(sock,bus):
                # A real physical leg disappears AFTER successful startup.
                bus.reject_off=True
                bus.dropped.add(13); deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    r=rpc(sock,'robot.morphology')
                    health=rpc(sock,'robot.health')
                    if r.get('fault_latched') and health.get('bus',{}).get('consecutive_errors',0)>=10: break
                    time.sleep(.1)
                assert r.get('fault_latched'),'missing leg was not latched as a fault'
                assert health.get('bus',{}).get('consecutive_errors',0)>=10,health
                assert not r['torque_off_acknowledged']
                assert rpc(sock,'robot.morphology.rearm',{})['accepted']
                until(sock,lambda r:r['rearm_result']=='refused')
                bus.dropped.remove(13)
                # Bus recovery must not clear the latch, and ack-only OFF fails
                # readback. A queued rearm is distinct from a cleared latch.
                time.sleep(.6);r=rpc(sock,'robot.morphology')
                assert r['fault_latched'] and not r['torque_off_acknowledged'],r
                bus.reject_off=False
                until(sock,lambda r:r['torque_off_acknowledged'])
                assert rpc(sock,'robot.morphology.rearm',{})['accepted']
                r=until(sock,lambda r:r['rearm_result']=='cleared')
                assert not r['fault_latched'],r
                assert not rpc(sock,'robot.enable',{'on':True})['accepted']
                assert not any(e[0]=='write' and e[2]==64 and e[3]==[1] for e in bus.events)


def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('--robotd',type=Path,required=True); ap.add_argument('--out',type=Path,default=Path('pty-results'))
    a=ap.parse_args()
    if not sys.platform.startswith('linux'): raise RuntimeError('PTY integration is Linux-only; use --fake smoke on macOS')
    binary=a.robotd.resolve()
    if not binary.is_file(): raise RuntimeError(f'missing native binary {binary}')
    a.out.mkdir(parents=True,exist_ok=True)
    for fast in [False,True]:
        for mouth,full,label in [(True,False,'mouth11'),(False,False,'legs10'),(True,True,'full15')]:
            case(binary,mouth,a.out/f'{label}-{"fast" if fast else "plain"}.json',fast,full)

if __name__=='__main__':
    try: main()
    except (OSError,RuntimeError,AssertionError,ValueError) as e:
        print(f'FAIL: {e}',file=sys.stderr); raise SystemExit(1)
