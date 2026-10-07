"""Shared observation engine and audited maintenance operations.

Only this local helper can open UART or manage the two allowlisted perception
services. HTTP never runs as root. Controllers/updater are observation targets,
never lifecycle targets; observation profiles never enter robotd's parameters.
"""
from __future__ import annotations
from collections import deque
import copy
import datetime as dt
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import shlex
import socket
import statistics
import struct
import subprocess
import threading
import time
import tomllib
import uuid
import wave
import zipfile

from protocol import (BUS_IDS, SERVO_IDS, SLOTS, MOUNT, ReadOnlyPort, imu_decode,
                      servo_decode, qnorm, qmul, qinv, rotate, euler)
from imu_quality import rotation_consistency

def kernel_uart(port):
    """Read driver counters without another UART open or driver reconfiguration.

    Counts are cumulative within this boot. Omitted fields remain unavailable,
    not measured zero. CRC rejects still apply even if a driver counter is zero.
    """
    name=Path(port).name;path=Path('/proc/tty/driver/serial')
    if not name.startswith('ttyS') or not name[4:].isdigit() or not path.exists():return {'available':False}
    try:
        line=next((s for s in path.read_text().splitlines() if s.startswith(name[4:]+':')),None)
        if not line:return {'available':False}
        counters={}
        for item in line.split():
            key,_,value=item.partition(':')
            if key in {'tx','rx','fe','oe','pe','brk','bo'} and value.isdigit():counters[key]=int(value)
        return {'available':True,'port':name,'raw_line':line,'counts':counters}
    except OSError as exc:return {'available':False,'error':str(exc)}

CONTROLLERS = ['robotd', 'zero3w-11servo-candidate']
WATCH_UNITS = CONTROLLERS + ['updaterd', 'tofd', 'mediad']
ALLOWED_UNITS = {'tofd', 'mediad'}
# User's temporary diagnostic waiver. Exclude ID14 requests so its timeout
# cannot block other devices; do not fabricate identity, angles or connected.
# This is never a robotd/control-policy change or a full 11-servo acceptance.
DIAGNOSTIC_EXEMPT_IDS = {14}

def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()

def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    with tmp.open('w') as f:
        json.dump(value, f, ensure_ascii=False, allow_nan=False, indent=2)
        f.flush(); os.fsync(f.fileno())
    os.chmod(tmp, 0o640); os.replace(tmp, path)

def boot_id():
    try: return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError: return 'local-test'

def service_state(unit):
    unit_name=unit if unit.endswith(('.service','.target','.mount','.slice','.socket')) else unit+'.service'
    r = subprocess.run(['systemctl', 'show', '--no-pager',
        '--property=LoadState,ActiveState,SubState,MainPID,UnitFileState,FragmentPath,DropInPaths,Wants,Requires,BindsTo,PartOf,Conflicts,Upholds,Conditions','--',unit_name],
        capture_output=True, text=True, timeout=3)
    row = dict(line.split('=', 1) for line in r.stdout.splitlines() if '=' in line)
    row['query_ok'] = r.returncode == 0
    if unit_name == 'robotd.service':
        r = subprocess.run(['busctl','get-property','org.freedesktop.systemd1',
            '/org/freedesktop/systemd1/unit/robotd_2eservice','org.freedesktop.systemd1.Unit','Conditions'],
            capture_output=True,text=True,timeout=3)
        fields = shlex.split(r.stdout)
        row['loaded_conditions'] = []
        if r.returncode == 0 and fields and fields[0] == 'a(sbbsi)':
            for i in range(int(fields[1])):
                values=fields[2+i*5:7+i*5]
                if len(values)==5: row['loaded_conditions'].append(values)
    row['fingerprints'] = {}
    for p in [row.get('FragmentPath', '')] + row.get('DropInPaths', '').split():
        if p and Path(p).is_file():
            row['fingerprints'][p] = hashlib.sha256(Path(p).read_bytes()).hexdigest()
    return row

def serial_owners(port):
    """Kernel exclusivity does not exclude another root opener: inspect actual FDs."""
    target = os.path.realpath(port); owners = set()
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name) == os.getpid(): continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    # proc fd links already name the opened device. Re-resolving
                    # every directory for every FD consumed the 20ms budget on
                    # Zero3W; readlink retains the actual-owner check cheaply.
                    if os.readlink(fd) == target: owners.add(int(proc.name))
                except OSError: pass
        except OSError: pass
    return sorted(owners)

def rpc(path, method, params=None):
    # A closed method list prevents this helper becoming a root robot-control proxy.
    if method not in {'hello', 'robot.health', 'robot.sound'}:
        raise ValueError('RPC方法不允许')
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(3); s.connect(str(path)); f = s.makefile('rwb')
        for ident, call, args in [(1, 'hello', {'api_version': 37}), (2, method, params or {})]:
            f.write((json.dumps({'jsonrpc': '2.0', 'id': ident, 'method': call, 'params': args})+'\n').encode()); f.flush()
            while True:
                line = f.readline(262145)
                if not line or len(line) > 262144: raise RuntimeError('IPC响应过长/断开')
                row = json.loads(line)
                if row.get('id') == ident: break
            if 'error' in row: raise RuntimeError(str(row['error']))
        return row.get('result', {})

class Engine:
    def __init__(self, data_dir, config='/etc/robot/robotd.toml', offline=False):
        self.root = Path(data_dir); self.root.mkdir(parents=True, exist_ok=True)
        for name in ['audit', 'profiles', 'records', 'exports', 'transactions']:
            (self.root/name).mkdir(exist_ok=True)
        self.lock = threading.RLock(); self.shutdown = threading.Event()
        self.bus_rescan=threading.Event()
        self.owner_checked=0.;self.owner_pids=[];self.owner_error=None
        self.commands = queue.Queue(maxsize=8); self.events = deque(maxlen=200)
        self.record_queue = queue.Queue(maxsize=1000); self.record_drops = 0
        self.samples = deque(maxlen=1000); self.summary = deque(maxlen=3600)
        self.latencies = deque(maxlen=3000); self.receives = {k: deque(maxlen=3000) for k in ['servo', 'imu', 'tof', 'camera']}
        self.offline = offline; self.boot = boot_id(); self.started = time.monotonic()
        self.config_path = Path(config)
        self.params = tomllib.loads(self.config_path.read_text()) if self.config_path.exists() else {}
        self.port_path = self.params.get('bus', {}).get('port', '/dev/ttyS2')
        self.profile = {'revision': 'factory', 'mount': MOUNT, 'bias': [0., 0., 0.], 'reference': [1., 0., 0., 0.]}
        if (self.root/'profile.json').exists(): self.profile = json.loads((self.root/'profile.json').read_text())
        wanted = {}
        if (self.root/'desired.json').exists(): wanted = json.loads((self.root/'desired.json').read_text())
        # Every process launch starts idle. Starting the station and starting
        # acquisition are separate explicit operations; old intent is history.
        self.observing = False
        self.bus_requested = False
        self.bus_hz = 20
        self.bus_fault=wanted.get('bus_fault')
        if self.bus_fault:self.bus_requested=False  # upgrades must not rearm a latched fault
        self.recording = False
        self.record_file = None; self.record_hour = None; self.record_cap = 2*1024**3
        self.record_bytes = sum(p.stat().st_size for p in (self.root/'records').glob('*.gz'))
        self.jobs = {}; self.audio_child = None; self.threads = []
        self.state = {'servo': {'devices': {}, 'source': '未验证', 'errors': 0},
            'imu': {'source': '未验证', 'errors': 0}, 'tof': {'source': 'tofd', 'errors': 0},
            'camera': {'source': 'mediad', 'errors': 0}, 'audio': {'state': '未验证', 'heard': None},
            'services': {}, 'service_checked_at': 0., 'audit_error': None, 'record_error': None,
            'bus_state': '被动订阅', 'fast_test': None, 'last_gap': None}
        self.last_error = {}
        self.load_audio_history()
        self.journal('startup', {'boot': self.boot, 'offline': offline, 'bus_requested': self.bus_requested})

    def load_audio_history(self):
        path=self.root/'audio-result.json'
        if path.exists():
            self.state['audio'].update(json.loads(path.read_text()),can_stop=False)
            if self.state['audio'].get('state')=='播放中':self.state['audio']['state']='上次播放结果未确认（进程重启）'
            return
        # Migrate the first candidate's already-audited test without replaying it.
        # Stream the journal so years of retained evidence do not become a cache.
        for p in sorted((self.root/'audit').glob('events-*.jsonl')):
            with p.open() as f:
                for line in f:
                    row=json.loads(line);action=row.get('action');result=row.get('result',{})
                    if row['kind']=='operation_finished' and action=='audio':
                        self.state['audio'].update(test_id=row['transaction'],state='历史请求已接受',heard=None,route=result.get('route'))
                    elif row['kind']=='audio_completed' and self.state['audio'].get('test_id'):
                        self.state['audio'].update(exit_code=row['exit_code'],state='播放器正常结束' if row['exit_code']==0 else '播放器失败')
                    elif row['kind']=='operation_finished' and action=='audio_heard' and self.state['audio'].get('test_id'):
                        self.state['audio']['heard']=result['heard']

    def save_audio(self):
        atomic_json(self.root/'audio-result.json',{k:v for k,v in self.state['audio'].items() if k not in ['pcm_owners']})

    def save_desired(self):
        # Save actual state, including fault/capacity stops, rather than restoring
        # the last click after an upgrade. A fault requires explicit observe-on.
        with self.lock:
            atomic_json(self.root/'desired.json',{'boot':self.boot,'observing':self.observing,
                'bus_requested':self.bus_requested,'recording':self.recording,'bus_fault':self.bus_fault})

    def journal(self, kind, data):
        row = {'utc': utc(), 'id': uuid.uuid4().hex, 'kind': kind, **data}
        path = self.root/'audit'/('events-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d')+'.jsonl')
        try:
            with path.open('a') as f:
                f.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n'); f.flush(); os.fsync(f.fileno())
            os.chmod(path, 0o640)
        except OSError as exc:
            self.state['audit_error'] = str(exc)
            raise RuntimeError('审计不可写；拒绝改变状态') from exc
        with self.lock: self.events.append(row)
        return row

    def error(self, component, exc):
        message = str(exc)
        with self.lock:
            self.state[component]['error'] = message
            self.state[component]['errors'] = self.state[component].get('errors', 0)+1
        if self.last_error.get(component) != message:
            self.last_error[component] = message
            try: self.journal('component_error', {'component': component, 'reason': message})
            except RuntimeError: pass

    def update(self, component, values, received=None):
        now = time.monotonic() if received is None else received
        with self.lock:
            self.state[component].update(values, received=now, error=None)
            self.receives[component].append(now)
            if component == 'imu': self.samples.append((now, copy.deepcopy(values)))
        if self.recording:
            recorded = copy.deepcopy(values)
            if component=='servo': recorded['devices']=copy.deepcopy(self.state['servo']['devices'])
            try: self.record_queue.put_nowait({'utc':utc(),'component':component,'values':recorded})
            except queue.Full: self.record_drops += 1

    def start(self):
        for fn in [self.maintenance, self.ownership_loop, self.bus_loop, self.tof_loop, self.robot_loop, self.operation_loop, self.record_loop]:
            t = threading.Thread(target=fn, daemon=True); self.threads.append(t); t.start()

    def close(self):
        self.shutdown.set(); self.stop_audio()
        for t in self.threads: t.join(timeout=4)
        if self.record_file: self.record_file.close(); self.record_file = None
        self.save_desired()
        self.journal('shutdown', {'bus_requested': self.bus_requested})

    def snapshot(self):
        with self.lock:
            out = copy.deepcopy(self.state); now = time.monotonic()
            for key, limit in [('servo', 2), ('imu', 1), ('tof', 1), ('camera', 3)]:
                comp = out[key]; stamp = comp.get('received')
                comp['age_ms'] = None if stamp is None else max(0., (now-stamp)*1000)
                comp['status'] = ('暂停' if not self.observing else '离线' if key=='servo' and comp.get('devices') and all(d.get('connected') is False for d in comp['devices'].values()) else '未验证' if stamp is None else
                    '离线' if comp.get('error') and now-stamp > limit else '过期' if now-stamp > limit else '实时')
                times = [v for v in self.receives[key] if now-v <= 10]
                comp['hz'] = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None
            for device in out['servo']['devices'].values():
                stamp = device.get('received')
                device['status'] = ('离线' if device.get('connected') is False else '实时' if stamp and now-stamp < 2 and self.observing else '过期' if stamp else '未验证')
                device['age_ms'] = (now-stamp)*1000 if stamp else None
                device['slow_age_ms'] = (now-device['slow_received'])*1000 if device.get('slow_received') else None
            for ident in DIAGNOSTIC_EXEMPT_IDS:
                device=out['servo']['devices'].setdefault(str(ident),{'id':ident,'slot':SLOTS[ident],'status':'未验证'})
                device.update(check_status='默认OK（用户暂时豁免）',exempt=True)
            imu = out['imu']
            # Fresh UART replies do not certify SFLP attitude quality. This
            # bounded diagnostic uses raw signals, never the saved display bias.
            imu['rotation_consistency'] = rotation_consistency(self.samples, now)
            if 'trunk_quat' in imu:
                # Service quaternions are already in trunk axes. Do not apply +90Y twice.
                sensor = imu.get('sensor_quat')
                trunk = qnorm(qmul(sensor, qinv(self.profile['mount']))) if sensor else imu['trunk_quat']
                gyro = rotate(self.profile['mount'], imu['gyro_sensor']) if 'gyro_sensor' in imu else imu.get('gyro', [0,0,0])
                imu.update(trunk_quat=trunk, euler=euler(trunk), gravity=rotate(qinv(trunk), [0.,0.,-1.]),
                    calibrated_quat=qnorm(qmul(self.profile['reference'], trunk)),
                    calibrated_gyro=[v-b for v,b in zip(gyro,self.profile['bias'])])
            latency = sorted(self.latencies)
            out.update(observing=self.observing, bus_requested=self.bus_requested, recording=self.recording,
                profile=copy.deepcopy(self.profile), uptime_s=now-self.started, boot=self.boot,
                session='observer-'+self.boot[:8], jobs=list(self.jobs.values())[-20:],
                events=list(self.events)[-25:], record_bytes=self.record_bytes, record_cap=self.record_cap,
                bus_p99_ms=latency[min(len(latency)-1, math.ceil(len(latency)*.99)-1)] if latency else None,
                bus_target_hz=self.bus_hz, diagnostic_exempt_ids=sorted(DIAGNOSTIC_EXEMPT_IDS), record_drops=self.record_drops, version=self.version())
            out['queue_sizes']={'action':self.commands.qsize(),'record':self.record_queue.qsize(),'events':len(self.events),'imu_samples':len(self.samples),'summary':len(self.summary)}
            out['ownership']={'age_ms':(now-self.owner_checked)*1000 if self.owner_checked else None,'pids':self.owner_pids,'error':self.owner_error}
            return out

    def version(self):
        p = Path(__file__).parent/'VERSION.json'
        return json.loads(p.read_text()) if p.exists() else {'version': 'working-tree'}

    def controllers_stopped(self):
        with self.lock:
            units = self.state['services']; fresh = time.monotonic()-self.state['service_checked_at'] < 2
            return fresh and all(units.get(n, {}).get('query_ok') and
                units[n].get('ActiveState') in ('inactive', 'failed') and units[n].get('MainPID') == '0' for n in CONTROLLERS)

    def controller_running(self):
        # Unknown/stale service state is not evidence of a running controller.
        # Wait for maintenance rather than counting missing sockets as IMU faults.
        with self.lock:
            units=self.state['services']
            return time.monotonic()-self.state['service_checked_at']<2 and any(
                units.get(n,{}).get('query_ok') and units[n].get('ActiveState')=='active'
                and units[n].get('MainPID','0')!='0' for n in CONTROLLERS)

    def controller_socket(self):
        with self.lock:
            units = self.state['services']
            if units.get('zero3w-11servo-candidate', {}).get('MainPID', '0') != '0': return '/run/zero3w-candidate.sock'
        return '/run/robotd.sock'

    def maintenance(self):
        while not self.shutdown.is_set():
            try:
                services = {n: service_state(n) for n in WATCH_UNITS} if not self.offline else {}
                with self.lock:
                    self.state.update(services=services, service_checked_at=time.monotonic())
                camera = Path('/run/mediad/camera.json')
                if not self.offline and camera.exists():
                    data = json.loads(camera.read_text())
                    # mtime is a statistics publication time, not an image arrival time.
                    age = max(0., time.time()-camera.stat().st_mtime)
                    self.update('camera', {'stats': data, 'evidence': '采集统计，图像接收由浏览器单独验证'}, time.monotonic()-age)
                self.inspect_audio()
                counters=kernel_uart(self.port_path)
                with self.lock:self.state['kernel_uart']=counters
                snap = self.snapshot()
                metrics = {'utc': utc(), 'uptime_s': snap['uptime_s'], 'bus_p99_ms': snap['bus_p99_ms'],
                    'rates': {k: snap[k]['hz'] for k in ['servo','imu','tof','camera']},
                    'errors': {k: snap[k]['errors'] for k in ['servo','imu','tof','camera']},
                    'resources': self.resources(),'kernel_uart':self.state.get('kernel_uart')}
                self.summary.append(metrics)
                with (self.root/'summary.jsonl').open('a') as f:
                    f.write(json.dumps(metrics, ensure_ascii=False)+'\n')
                os.chmod(self.root/'summary.jsonl', 0o640)
            except Exception as exc:
                with self.lock: self.state['record_error'] = str(exc)
            self.shutdown.wait(1 if self.observing else 5)

    def resources(self):
        row = {'load': list(os.getloadavg()), 'disk_free': __import__('shutil').disk_usage(self.root).free}
        now=time.monotonic(); usage=os.times(); cpu=usage.user+usage.system+usage.children_user+usage.children_system
        if hasattr(self,'resource_previous'):
            then,last=self.resource_previous;row['cpu_percent']=(cpu-last)/(now-then)*100
        self.resource_previous=(now,cpu)
        stat = Path('/proc/self/status')
        if stat.exists():
            row.update({line.split(':')[0]: line.split(':',1)[1].strip() for line in stat.read_text().splitlines() if line.startswith(('VmRSS:', 'VmSize:'))})
        return row

    def ownership_loop(self):
        """Measure owners outside the 20ms acquisition budget; stale data blocks IO.

        On this board proc scanning takes ~100ms in Python and ~65ms in native
        fuser. A separate native worker keeps that cost/GIL out of packet timing.
        Kernel TIOCEXCL/flock still guard the open; owner changes are acted on at
        the next bus iteration after discovery. Controller state is checked too.
        """
        while not self.shutdown.is_set():
            if self.offline or not self.observing or not self.bus_requested:
                self.owner_checked=0.;self.shutdown.wait(.1);continue
            start=time.monotonic()
            try:
                r=subprocess.run(['fuser','--',self.port_path],capture_output=True,text=True,timeout=.4)
                if r.returncode not in [0,1]:raise RuntimeError('fuser无法检查UART所有权')
                pids=[int(v) for v in r.stdout.split()]
                self.owner_pids=[pid for pid in pids if pid!=os.getpid()];self.owner_error=None;self.owner_checked=time.monotonic()
            except Exception as exc:self.owner_error=str(exc);self.owner_checked=0.
            # Halve expensive fuser scans while retaining the existing 500ms
            # freshness gate and immediate close when an owner is detected.
            self.shutdown.wait(max(.01,.3-(time.monotonic()-start)))

    def ownership_ready(self):
        return bool(self.owner_checked and time.monotonic()-self.owner_checked<.5 and not self.owner_error and not self.owner_pids)

    def record_loop(self):
        while not self.shutdown.is_set():
            try: row=self.record_queue.get(timeout=.5)
            except queue.Empty: continue
            try:
                if self.recording: self.write_record(row)
            except Exception as exc:
                self.recording=False; self.state['record_error']=str(exc)
            finally: self.record_queue.task_done()

    def write_record(self, row):
        # Raw records are a separate capacity budget; filling it must not stop monitoring.
        if self.record_bytes >= self.record_cap:
            self.recording = False; self.state['record_error'] = '原始记录达到2GiB上限；观测继续'
            self.journal('record_capacity', {'bytes': self.record_bytes}); return
        hour = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d-%H')
        if hour != self.record_hour:
            if self.record_file: self.record_file.close()
            self.record_hour = hour
            self.record_path = self.root/'records'/f'{hour}-{uuid.uuid4().hex[:8]}.jsonl.gz'
            self.record_file = gzip.open(self.record_path, 'at', encoding='utf-8'); os.chmod(self.record_path, 0o640)
        before = self.record_path.stat().st_size
        self.record_file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n'); self.record_file.flush()
        self.record_bytes += max(0, self.record_path.stat().st_size-before)

    def bus_loop(self):
        port = None; verified=False; failures = 0; slow_index = 0; last_slow = 0.; target = time.monotonic()
        while not self.shutdown.is_set():
            try:
                wanted = self.observing and self.bus_requested and not self.offline and self.controllers_stopped()
                if self.bus_rescan.is_set():
                    if port:port.close();port=None
                    failures=0  # explicit observe-on begins a new verified attempt
                    self.bus_rescan.clear()
                if not wanted:
                    if port: port.close(); port = None
                    self.state['bus_state'] = ('故障锁止：'+self.bus_fault+'；需手动开启') if self.bus_fault else '服务订阅' if not self.controllers_stopped() else '未开启独立总线'
                    self.shutdown.wait(.1); continue
                if self.owner_error:raise RuntimeError('UART所有权检查失败: '+self.owner_error)
                if self.owner_pids:raise RuntimeError('UART由其他进程持有: '+str(self.owner_pids))
                if not self.ownership_ready():
                    if port:port.close();port=None
                    self.state['bus_state']='等待新鲜UART所有权检查';self.shutdown.wait(.02);continue
                if port is None:
                    verified=False
                    port = ReadOnlyPort(self.port_path)
                    metadata = {};bus_ids=[]
                    for ident in BUS_IDS:
                        if ident in DIAGNOSTIC_EXEMPT_IDS:
                            metadata[str(ident)]={'id':ident,'slot':SLOTS[ident],'exempt':True}
                            continue
                        if not self.controllers_stopped() or not self.ownership_ready():raise RuntimeError('身份查询期间UART所有权变化')
                        try:raw, _ = port.identity(ident)
                        except TimeoutError as exc:
                            # A diagnostics portal still observes a responding IMU
                            # when servo power is off. Never invent missing leg data.
                            if ident in SERVO_IDS:metadata[str(ident)]={'id':ident,'slot':SLOTS[ident],'connected':False,'error':str(exc)}
                            else:self.error('imu',exc)
                            continue
                        model = int.from_bytes(raw[:2], 'little')
                        if ident == 200:
                            if (model, raw[2]) != (1030, 3): raise RuntimeError(f'IMU身份未核验: {model}/FW{raw[2]}')
                            self.state['imu']['identity'] = {'id':200, 'model':model, 'firmware':raw[2]}
                        else:
                            if model != 1200: raise RuntimeError(f'ID{ident}型号未核验: {model}')
                            data, status = port.read(ident, 64, 7)
                            if data[0] != 0: raise RuntimeError(f'ID{ident}扭矩非零；只读采集未开启')
                            metadata[str(ident)] = {'id':ident, 'slot':SLOTS[ident], 'model':model, 'firmware':raw[2],
                                'connected':True,'torque':data[0], 'hardware_error':data[6], 'status_byte':status, 'slow_received':time.monotonic()}
                        bus_ids.append(ident)
                    if not bus_ids:raise RuntimeError('没有真实器件应答，需检查连接后重新开启')
                    with self.lock: self.state['servo']['devices'] = metadata
                    self.journal('bus_acquired', {'port':self.port_path, 'ids':bus_ids,'missing':[i for i in BUS_IDS if i not in bus_ids], 'writes':False})
                    connected_servos=[i for i in bus_ids if i in SERVO_IDS]
                    verified=True
                    target = time.monotonic()
                if not self.controllers_stopped(): raise RuntimeError('控制器状态变化，释放UART')
                begin = time.monotonic(); rows = port.sync(ids=bus_ids); received = time.monotonic()
                devices = {}; imu = None
                for ident, status, raw in rows:
                    if status & 0x7f or len(raw) != 12: raise ValueError('设备错误/短块 ID'+str(ident))
                    if ident == 200: imu = imu_decode(raw); imu['source'] = '独立普通Sync Read'
                    else: devices[str(ident)] = {**servo_decode(ident, raw, status), 'received':received}
                with self.lock:
                    for ident, data in devices.items(): self.state['servo']['devices'].setdefault(ident, {}).update(data)
                if devices:self.update('servo', {'source':'独立普通Sync Read'}, received)
                if imu:self.update('imu', imu, received)
                self.latencies.append((received-begin)*1000)
                self.state['bus_state'] = f'只读采集中：舵机 {len(connected_servos)}/10 · ID14豁免 · {self.bus_hz}Hz · IMU {"有" if 200 in bus_ids else "无"}'; failures = 0
                # Spread slow telemetry over the second, rather than a 33-read burst.
                if connected_servos and received-last_slow >= .09:
                    ident = connected_servos[slow_index % len(connected_servos)]; slow_index += 1; last_slow = received
                    data, status = port.read(ident, 144, 3); flags, _ = port.read(ident, 64, 7)
                    if flags[0]: raise RuntimeError(f'ID{ident}扭矩状态变化，释放UART')
                    with self.lock:
                        self.state['servo']['devices'][str(ident)].update(voltage=int.from_bytes(data[:2],'little')*.1,
                            temperature=data[2], torque=flags[0], hardware_error=flags[6], slow_received=time.monotonic(),
                            voltage_condition='用户已接受约7.4V；原始告警保留')
                target += 1/self.bus_hz; delay = target-time.monotonic()
                if delay > 0: self.shutdown.wait(delay)
                else: target = time.monotonic()
            except Exception as exc:
                self.error('servo', exc); self.error('imu', exc); failures += 1
                recoverable=verified and port is not None and isinstance(exc,(TimeoutError,ValueError)) and failures<3
                # A bad frame is discarded, never corrected into telemetry. Keep
                # the verified exclusive port for a bounded retry, so isolated
                # CRC errors do not cause an identity scan + 0.5s outage. Do not
                # reset the failure streak at open: only a valid sync resets it.
                # Ownership/identity/torque errors still close and require rearm.
                if not recoverable:
                    if port:port.close();port=None
                    self.bus_requested = False;self.bus_fault=str(exc);self.state['bus_state'] = '需手动重新开启'
                    try:
                        self.save_desired()
                        self.journal('bus_latched',{'reason':self.bus_fault,'manual_rearm_required':True})
                    except OSError as persistence:
                        self.state['record_error']='无法保存故障锁止：'+str(persistence)
                    except RuntimeError:pass  # journal already exposes audit_error; UART stays closed
                self.shutdown.wait(.02 if recoverable else .5)
        if port: port.close()

    def stream(self, path, method, component):
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(2); s.connect(path); f = s.makefile('rwb')
            # tofd's documented stream contract has no hello route. Controllers
            # do negotiate; log their skew but never refuse solely on a version.
            calls=([(1,'hello',{'api_version':37})] if component=='imu' else [])+[(2,method,{'hz':50} if component=='imu' else {})]
            for ident, call, args in calls:
                f.write((json.dumps({'jsonrpc':'2.0','id':ident,'method':call,'params':args})+'\n').encode()); f.flush()
            while not self.shutdown.is_set() and self.observing:
                if component == 'imu' and not self.controller_running(): return
                line = f.readline(262145)
                if not line or len(line)>262144: raise RuntimeError('数据流断开/过长')
                obj = json.loads(line)
                if 'error' in obj: raise RuntimeError(str(obj['error']))
                if component == 'tof' and obj.get('method') == 'tof.frame':
                    data = obj['params']
                    if data.get('rows') != 8 or data.get('cols') != 8 or len(data.get('distance_mm',[])) != 64 or len(data.get('status',[])) != 64:
                        raise ValueError('ToF不是有效8×8帧')
                    distances = data['distance_mm']; statuses = data['status']
                    if not all(isinstance(v,int) for v in distances+statuses): raise ValueError('ToF类型错误')
                    data['zone_status'] = data.pop('status')
                    data['valid'] = [st in (5,9) and d>0 for st,d in zip(statuses,distances)]
                    data['source'] = 'tofd 实测'; self.update('tof', data)
                elif component == 'imu' and obj.get('method') == 'robot.state':
                    data = obj['params']; joints = data.get('joints',[])
                    if len(joints)!=15 or not all(math.isfinite(x) for x in joints): raise ValueError('逻辑关节契约错误')
                    devices = {}
                    for ident in SERVO_IDS:
                        slot = SLOTS[ident]; row = {'id':ident,'slot':slot,'position':joints[slot], 'received':time.monotonic()}
                        for field, key in [('velocities','velocity'), ('currents_ma','current_ma')]:
                            values = data.get(field,[])
                            if len(values)==15 and math.isfinite(values[slot]): row[key]=values[slot]
                        devices[str(ident)] = row
                    self.update('servo', {'devices':devices, 'source':'robot.state 遥测（不是独立Ping）'})
                    if data.get('imu'):
                        imu=data['imu']; quat=qnorm(imu['quat']); gyro=imu['gyro']
                        if len(gyro)!=3 or not all(math.isfinite(x) for x in gyro): raise ValueError('非有限角速度')
                        # The baseline always applies DEFAULT_MOUNT; reconstruct sensor frame
                        # for display-only mounting changes without modifying the controller.
                        self.update('imu', {'trunk_quat':quat, 'sensor_quat':qmul(quat,MOUNT), 'gyro':gyro,
                            'gyro_sensor':rotate(qinv(MOUNT),gyro), 'source':'robot.state 已应用安装旋转',
                            'freshness':'控制器遥测；融合就绪以robot.health为准'})

    def tof_loop(self):
        while not self.shutdown.is_set():
            if self.offline or not self.observing: self.shutdown.wait(.5); continue
            try: self.stream('/run/tofd/tof.sock', 'tof.stream', 'tof')
            except Exception as exc: self.error('tof', exc); self.shutdown.wait(2)

    def robot_loop(self):
        while not self.shutdown.is_set():
            if self.offline or not self.observing or not self.controller_running(): self.shutdown.wait(.5); continue
            try: self.stream(self.controller_socket(), 'robot.subscribe', 'imu')
            except Exception as exc: self.error('imu', exc); self.shutdown.wait(2)

    def submit(self, action, args):
        if action not in {'observe','record','service','restore','calibrate','audio','audio_stop','audio_heard','fast_test','export'}:
            raise ValueError('操作不允许')
        if self.state['audit_error']: raise RuntimeError('审计故障，拒绝操作')
        ident = uuid.uuid4().hex
        self.journal('operation_queued', {'transaction':ident,'action':action,'args':args})
        with self.lock:
            if len(self.jobs) >= 100:
                oldest = next((k for k,v in self.jobs.items() if v['status'] not in ['排队','执行中']), None)
                if oldest: del self.jobs[oldest]
            self.jobs[ident] = {'id':ident,'action':action,'status':'排队','started':utc()}
        try: self.commands.put_nowait((ident,action,args))
        except queue.Full:
            self.jobs[ident].update(status='失败',error='操作队列忙'); raise RuntimeError('操作队列忙')
        return {'id':ident,'status':'排队'}

    def operation_loop(self):
        while not self.shutdown.is_set():
            try: ident, action, args = self.commands.get(timeout=.5)
            except queue.Empty: continue
            self.jobs[ident]['status']='执行中'
            try:
                result = self.perform(ident,action,args)
                self.journal('operation_finished', {'transaction':ident,'action':action,'result':result})
                self.jobs[ident].update(status='完成', result=result)
            except Exception as exc:
                self.jobs[ident].update(status='失败', error=str(exc))
                try: self.journal('operation_failed', {'transaction':ident,'action':action,'reason':str(exc)})
                except RuntimeError: pass
            finally: self.commands.task_done()

    def perform(self, ident, action, args):
        if action in {'observe','record'}:
            if set(args)-({'enabled','hz'} if action=='observe' else {'enabled'}) or 'enabled' not in args or not isinstance(args['enabled'],bool): raise ValueError('enabled必须为布尔')
            if action=='observe':
                hz=args.get('hz',20)
                if type(hz)!=int or hz not in [20,50]:raise ValueError('诊断采集仅支持20/50Hz')
                self.bus_hz=hz
                self.observing=args['enabled']; self.bus_requested=args['enabled']
                if args['enabled']:self.bus_fault=None;self.bus_rescan.set()
            else: self.recording=args['enabled']
            self.save_desired()
            return {'observing':self.observing,'bus_requested':self.bus_requested,'recording':self.recording,'bus_target_hz':self.bus_hz,'diagnostic_exempt_ids':sorted(DIAGNOSTIC_EXEMPT_IDS)}
        if action=='service':
            if set(args)!={'unit','verb'}: raise ValueError('服务参数不允许')
            return self.change_service(ident,args['unit'],args['verb'])
        if action=='restore':
            if set(args)!={'transaction'}: raise ValueError('恢复参数错误')
            tid=args['transaction']
            if len(tid)!=32 or any(c not in '0123456789abcdef' for c in tid): raise ValueError('事务ID错误')
            tx=json.loads((self.root/'transactions'/f'{tid}.json').read_text())
            now=service_state(tx['unit'])
            if self.boot != tx['boot'] or not tx.get('after') or any(now.get(k)!=tx['after'].get(k) for k in ['ActiveState','MainPID','UnitFileState','fingerprints']):
                raise RuntimeError('服务已有后续变化；拒绝覆盖')
            return self.change_service(ident,tx['unit'],'start' if tx['before']['ActiveState']=='active' else 'stop')
        if action=='calibrate': return self.calibrate(ident,args)
        if action=='audio':
            try:result=self.play_audio(args)
            except Exception as exc:
                self.state['audio'].update(test_id=None,state='测试失败',error=str(exc));self.save_audio();raise
            self.state['audio'].update(test_id=ident,error=None);self.save_audio();return result
        if action=='audio_stop':
            if args: raise ValueError('停止播放没有参数')
            return self.stop_audio()
        if action=='audio_heard':
            if set(args)!={'heard'} or not isinstance(args['heard'],bool): raise ValueError('heard必须为布尔')
            if not self.state['audio'].get('test_id'): raise RuntimeError('需要先完成一次声音测试请求')
            self.state['audio']['heard']=args['heard'];self.save_audio();return {'heard':args['heard'],'evidence':'现场人工确认'}
        if action=='fast_test':
            if args: raise ValueError('Fast诊断没有可调指令')
            if DIAGNOSTIC_EXEMPT_IDS:
                result={'pass':None,'status':'未执行','reason':'ID14暂时豁免；完整Fast诊断需要全部11舵机，不阻塞其他观测'}
                self.state['fast_test']=result;return result
            if self.offline or self.bus_requested or not self.controllers_stopped(): raise RuntimeError('先停止采集，且控制器必须停止')
            if serial_owners(self.port_path): raise RuntimeError('串口被占用')
            port=ReadOnlyPort(self.port_path)
            try:
                for sid in SERVO_IDS:
                    raw,_=port.read(sid,64,7)
                    if raw[0]: raise RuntimeError('扭矩非零')
                samples=[]
                for _ in range(20):
                    if not self.controllers_stopped() or serial_owners(self.port_path): raise RuntimeError('串口所有权变化')
                    begin=time.monotonic(); rows=port.sync(True)
                    if any(st&0x7f or len(data)!=12 for _,st,data in rows): raise ValueError('Fast状态/长度错误')
                    # Select by device identity: the baseline puts IMU200 first,
                    # so the final response is a right-leg servo, not the IMU.
                    imu_decode(next(data for sid,_,data in rows if sid==200))
                    samples.append((time.monotonic()-begin)*1000)
                result={'pass':True,'count':20,'latencies_ms':samples,'ids':BUS_IDS}
            except Exception as exc: result={'pass':False,'error':str(exc)}
            finally: port.close()
            self.state['fast_test']=result; return result
        if action=='export':
            if args: raise ValueError('导出没有路径参数')
            if self.record_file: self.record_file.flush()
            output=self.root/'exports'/f'{ident}.zip'
            with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED) as z:
                z.writestr('status.json',json.dumps(self.snapshot(),ensure_ascii=False,indent=2))
                z.writestr('summary.csv','utc,uptime_s,servo_hz,imu_hz,tof_hz,bus_p99_ms\n'+''.join(
                    f"{r['utc']},{r['uptime_s']},{r['rates']['servo']},{r['rates']['imu']},{r['rates']['tof']},{r['bus_p99_ms']}\n" for r in self.summary))
                for directory in ['audit','profiles','transactions']:
                    for p in (self.root/directory).glob('*'):
                        if p.is_file(): z.write(p,p.relative_to(self.root))
                for name in ['profile.json','desired.json','summary.jsonl']:
                    p=self.root/name
                    if p.exists(): z.write(p,name)
            os.chmod(output,0o640)
            return {'download':'/api/download/'+ident+'.zip','includes':'状态、CSV、完整操作审计、校正历史；原始记录单独归档'}

    def command(self, argv, ident):
        self.journal('command_started', {'transaction':ident,'argv':argv})
        r=subprocess.run(argv,capture_output=True,text=True,timeout=25)
        self.journal('command_finished', {'transaction':ident,'argv':argv,'exit_code':r.returncode,
            'stdout':r.stdout,'stderr':r.stderr})
        if r.returncode: raise RuntimeError(r.stderr.strip() or '服务操作失败')

    def can_start_media(self, states):
        media=states['mediad']
        dependencies=set((media.get('Wants','')+' '+media.get('Requires','')).split())
        # Validate every existing stop condition rather than removing dependencies or
        # creating approval markers. Refuse unexpected transitive controller pulls.
        for name in CONTROLLERS:
            if name+'.service' not in dependencies: continue
            state=states[name]
            if state.get('ActiveState')=='active' and state.get('MainPID')!='0': continue
            if name!='robotd': raise RuntimeError('摄像头可能启动候选控制器')
            hold='/etc/systemd/system/robotd.service.d/91-zero3w-hold.conf'
            expected='/etc/robot/ZERO3W_FULL15_RESTORED'
            p=Path(hold)
            if hold not in state.get('fingerprints',{}) or not p.exists() or Path(expected).exists():
                raise RuntimeError('摄像头依赖可能启动robotd；停止保护无效')
            text=p.read_text()
            if 'ConditionPathExists='+expected not in text or '!'+expected in text:
                raise RuntimeError('robotd停止保护内容变化')
            conditions=state.get('loaded_conditions',[])
            if not any(row[:4]==['ConditionPathExists','false','false',expected] for row in conditions):
                raise RuntimeError('停止保护未作为非反转、非触发条件加载到有效unit')
        # configd/updater dependency chains must also be inspected before installation;
        # a new unexpected requirement at runtime is a configuration drift, not permission.
        for field in ['Requires','BindsTo','PartOf','Conflicts']:
            if any(n+'.service' in media.get(field,'').split() for n in CONTROLLERS+['updaterd']):
                raise RuntimeError('媒体unit存在未支持的控制器依赖: '+field)

    def guard_dependencies(self, unit, states):
        """Follow inactive dependency jobs: an indirect Wants is still a start path."""
        protected={n+'.service' for n in CONTROLLERS+['updaterd']}
        seen=set(); pending=[unit+'.service']
        while pending:
            name=pending.pop()
            if name in seen:continue
            seen.add(name)
            if len(seen)>128:raise RuntimeError('服务依赖图过大，拒绝状态切换')
            row=states.get(name.removesuffix('.service')) or service_state(name)
            if not row.get('query_ok'):raise RuntimeError('依赖状态不可读 '+name)
            if name in protected:
                if row.get('ActiveState')=='active':continue
                if name=='robotd.service':
                    self.can_start_media({**states,'mediad':{'Wants':'robotd.service'},'robotd':row})
                    continue
                raise RuntimeError('感知服务可能启动受保护单元 '+name)
            if name!=unit+'.service' and row.get('ActiveState')=='active':continue
            for other in row.get('Conflicts','').split():
                if other in protected:raise RuntimeError('感知依赖可能停止受保护单元 '+other)
            for field in ['Wants','Requires','BindsTo','Upholds']:pending.extend(row.get(field,'').split())

    def change_service(self, ident, unit, verb):
        if unit not in ALLOWED_UNITS or verb not in {'start','stop','restart'}: raise ValueError('只允许tofd/mediad的启停重启')
        if self.offline: raise RuntimeError('离线模式不操作真实服务')
        before={n:service_state(n) for n in WATCH_UNITS}
        if not all(r.get('query_ok') for r in before.values()): raise RuntimeError('服务状态无法确认')
        if unit=='mediad' and verb!='stop': self.can_start_media(before)
        if verb!='stop':self.guard_dependencies(unit,before)
        # A stop can propagate through reverse Requires/PartOf, so refuse that too.
        for name in CONTROLLERS+['updaterd']:
            if any(unit+'.service' in before[name].get(k,'').split() for k in ['Requires','BindsTo','PartOf']):
                raise RuntimeError('停止感知服务可能影响控制器: '+name)
        tx={'id':ident,'unit':unit,'verb':verb,'boot':self.boot,'before':before[unit],'protected_before':{n:before[n] for n in CONTROLLERS+['updaterd']}}
        path=self.root/'transactions'/f'{ident}.json'; atomic_json(path,tx)
        started=time.monotonic(); started_wall=time.time(); self.state['last_gap']={'component':unit,'started_utc':utc(),'status':'切换中'}
        try:
            self.command(['systemctl',verb,unit+'.service'],ident)
            after=service_state(unit); tx['after']=after; atomic_json(path,tx)
            for name in CONTROLLERS+['updaterd']:
                now=service_state(name)
                if any(now.get(k)!=before[name].get(k) for k in ['ActiveState','MainPID','UnitFileState','fingerprints']):
                    raise RuntimeError('受保护服务发生变化: '+name+'；保留证据，不自动干预控制器')
            if verb!='stop':
                deadline=time.monotonic()+20
                while time.monotonic()<deadline:
                    if unit=='tofd': fresh=self.state['tof'].get('received',0)>started
                    else:
                        p=Path('/run/mediad/camera.json'); fresh=p.exists() and p.stat().st_mtime>started_wall
                    if unit=='tofd' and not self.observing:
                        with socket.socket(socket.AF_UNIX) as s:
                            s.settimeout(2);s.connect('/run/tofd/tof.sock');f=s.makefile('rwb')
                            for msg in [{'jsonrpc':'2.0','id':2,'method':'tof.stream','params':{}}]:
                                f.write((json.dumps(msg)+'\n').encode())
                            f.flush()
                            for _ in range(8):
                                line=f.readline(262145)
                                if not line or len(line)>262144:raise RuntimeError('ToF恢复验证断开')
                                obj=json.loads(line)
                                if obj.get('method')=='tof.frame':
                                    data=obj['params'];fresh=data.get('rows')==8 and data.get('cols')==8 and len(data.get('distance_mm',[]))==64
                                    break
                    if fresh: break
                    self.shutdown.wait(.2)
                else: raise RuntimeError('服务启动但20秒内未取得新的采集证据')
            self.state['last_gap'].update(status='已恢复' if verb!='stop' else '已停止',duration_s=time.monotonic()-started)
            return {'transaction':ident,'unit':unit,'verb':verb,'after':after,'gap':self.state['last_gap']}
        except Exception as exc:
            tx['after']=service_state(unit); tx['error']=str(exc); atomic_json(path,tx)
            self.state['last_gap'].update(status='恢复失败',reason=str(exc)); raise

    def calibrate(self, ident, args):
        mode=args.get('mode')
        if mode not in {'mount','bias','yaw','reference','undo'}: raise ValueError('校正方式错误')
        if set(args)-{'mode','mount'}: raise ValueError('校正参数不允许')
        previous=copy.deepcopy(self.profile); profile=copy.deepcopy(previous)
        if mode=='undo':
            if not previous.get('previous'): raise RuntimeError('没有可撤销的校正')
            profile=json.loads((self.root/'profiles'/f"{previous['previous']}.json").read_text())
        elif mode=='mount':
            mount=args.get('mount')
            if not isinstance(mount,list): raise ValueError('安装四元数必须为数组')
            profile.update(mount=qnorm(mount),reference=[1.,0.,0.,0.],bias=[0.,0.,0.])
        else:
            imu=self.snapshot()['imu']
            if imu['rotation_consistency']['status']=='不一致':
                raise RuntimeError('融合姿态与角速度不一致；先排查IMU，归零或显示偏置不能修复融合')
            # At the default 20Hz, a fixed 25 samples/second gate can never
            # succeed. Require 15 recent samples for diagnostics; retain the
            # original 25 at 50Hz, the 1s freshness and all movement guards.
            minimum=15 if self.bus_hz==20 else 25
            if imu['status']!='实时' or not imu.get('trunk_quat') or len(self.samples)<minimum: raise RuntimeError(f'需要至少{minimum}个新鲜有效IMU样本')
            recent=[r for t,r in self.samples if t>=time.monotonic()-1]
            if len(recent)<minimum: raise RuntimeError(f'近1秒有效样本不足{minimum}')
            anchor=recent[0]['trunk_quat']
            if any(math.sqrt(sum(v*v for v in r['gyro']))>math.radians(5) or
                   math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(anchor,r['trunk_quat']))))))>1 for r in recent):
                raise RuntimeError('检测到运动，参考校正未保存')
            if mode=='bias':
                begin=time.monotonic()
                while time.monotonic()-begin<10:
                    if self.shutdown.wait(.1): raise RuntimeError('校正已中断')
                    if self.snapshot()['imu']['status']!='实时': raise RuntimeError('采样期间IMU过期')
                samples=[r for t,r in self.samples if t>=begin]
                if len(samples)<100: raise RuntimeError('静止校正样本不足100')
                gyros=[rotate(profile['mount'],r['gyro_sensor']) if 'gyro_sensor' in r else r['gyro'] for r in samples]
                if any(math.sqrt(sum(v*v for v in g))>math.radians(5) for g in gyros): raise RuntimeError('检测到运动，偏置未保存')
                if any(statistics.pstdev([g[i] for g in gyros])>math.radians(.5) for i in range(3)): raise RuntimeError('角速度不稳定')
                first=samples[0].get('trunk_quat')
                if first and any(math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(first,r['trunk_quat']))))))>1 for r in samples):
                    raise RuntimeError('姿态变化超过1°，偏置未保存')
                profile['bias']=[statistics.mean(g[i] for g in gyros) for i in range(3)]
                profile['sample_count']=len(samples)
            elif mode=='yaw':
                yaw=euler(imu['trunk_quat'])[2]; profile['reference']=[math.cos(yaw/2),0.,0.,-math.sin(yaw/2)]
            else: profile['reference']=qinv(imu['trunk_quat'])
        if previous['revision']=='factory': atomic_json(self.root/'profiles'/'factory.json',previous)
        parent=previous['revision'];older=profile.get('previous') if mode=='undo' else parent
        profile.update(revision=ident,previous=older,parent=parent,utc=utc(),scope='仅门户观测，不修改控制器或固件')
        atomic_json(self.root/'profiles'/f'{ident}.json',profile); atomic_json(self.root/'profile.json',profile)
        self.profile=profile; return profile

    def inspect_audio(self):
        audio=self.params.get('audio',{}); bank=Path(audio.get('bank','/var/lib/robot/sounds'))
        owners=[]
        if not self.offline:
            for proc in Path('/proc').glob('[0-9]*'):
                if int(proc.name)==os.getpid() or (self.audio_child and int(proc.name)==self.audio_child.pid): continue
                try:
                    if any('/dev/snd/pcm' in os.readlink(fd) and os.readlink(fd).endswith('p') for fd in (proc/'fd').iterdir()): owners.append(int(proc.name))
                except OSError: pass
        with self.lock: self.state['audio'].update(device=audio.get('device','plughw:aic3104'),bank=str(bank),
            bank_count=len(list(bank.glob('chirp*.wav'))),pcm_owners=owners,enabled=audio.get('enabled',True))
        if self.audio_child and self.audio_child.poll() is not None:
            code=self.audio_child.returncode; self.audio_child=None
            self.state['audio'].update(state='播放器正常结束' if code==0 else '播放器失败',exit_code=code,can_stop=False)
            self.journal('audio_completed', {'exit_code':code,'heard':self.state['audio'].get('heard')})
            self.save_audio()

    def play_audio(self,args):
        if set(args)-{'level'}: raise ValueError('声音参数不允许')
        level=args.get('level',.1)
        if isinstance(level,bool) or not isinstance(level,(int,float)) or not math.isfinite(level) or not .01<=level<=.3:
            raise ValueError('数字幅度限于1%–30%')
        if self.offline: raise RuntimeError('离线模式只提供浏览器试听')
        self.inspect_audio(); audio=self.state['audio']
        if audio['pcm_owners'] or self.audio_child: raise RuntimeError('声卡忙；未中断其他播放器')
        audio['heard']=None
        if not self.controllers_stopped():
            health=rpc(self.controller_socket(),'robot.health')
            loop=health.get('control_loop') or {}
            if not health.get('healthy') or not loop.get('ticks') or loop.get('last_tick_age_ms',9999)>100:
                raise RuntimeError('控制器音频通路没有健康控制循环；未启动控制器')
            answer=rpc(self.controller_socket(),'robot.sound',{'tag':'chirp'})
            self.journal('audio_service_rpc',{'method':'robot.sound','params':{'tag':'chirp'},'response':answer})
            if not answer.get('accepted'): raise RuntimeError(answer.get('reason','原服务拒绝声音请求'))
            audio.update(state='请求已接受；现场听音待确认',route='原robot.sound',level='原服务音量',can_stop=False)
            return {'accepted':True,'completed':None,'heard':None,'route':'robot.sound','note':'短音自然结束，现有接口未提供播放完成回执'}
        if not audio['enabled']: raise RuntimeError('实际配置禁用了音频')
        candidates=sorted(Path(audio['bank']).glob('chirp*.wav'))
        if not candidates:
            private=Path(__file__).parent/'static'/'chirp-reference.wav'
            if not private.exists(): raise RuntimeError('原音库和门户私有测试音均缺失')
            candidates=[private]; audio['bank_source']='门户私有测试音：由仓库sounds render生成，原音库未改写'
        else: audio['bank_source']='已有机器人音库'
        # Reuse bank timbre but scale a private copy; no persistent amixer changes and
        # no `sounds play` default-device fallback are permitted.
        wav=self.root/'audio-test.wav'
        with wave.open(str(candidates[0]),'rb') as f:
            if f.getsampwidth()!=2 or f.getframerate()!=48000 or f.getnchannels()>2: raise RuntimeError('音库必须为48kHz/16bit PCM')
            channels=f.getnchannels(); raw=f.readframes(min(f.getnframes(),96000))
        values=struct.unpack('<'+'h'*(len(raw)//2),raw)
        with wave.open(str(wav),'wb') as out:
            out.setnchannels(channels); out.setsampwidth(2); out.setframerate(48000)
            out.writeframes(struct.pack('<'+'h'*len(values),*(round(v*level) for v in values)))
        argv=['aplay','-q','-D',audio['device'],str(wav)]
        self.journal('audio_player_started',{'argv':argv,'source_sha256':hashlib.sha256(candidates[0].read_bytes()).hexdigest(),'level':level})
        log=(self.root/'audit'/'audio-player.log').open('ab')
        try: self.audio_child=subprocess.Popen(argv,stdout=log,stderr=log,start_new_session=True)
        finally: log.close()
        os.chmod(self.root/'audit'/'audio-player.log',0o640)
        child=self.audio_child
        audio.update(state='播放中',route='独立短音播放器',level=level,can_stop=True,exit_code=None)
        def watchdog():
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try: os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError: pass
                self.journal('audio_timeout',{'pid':child.pid})
        threading.Thread(target=watchdog,daemon=True).start()
        return {'accepted':True,'completed':None,'heard':None,'route':'owned-aplay','max_duration_s':2,'level':level}

    def stop_audio(self):
        child=self.audio_child
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try: child.wait(timeout=1)
            except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait(timeout=1)
            self.audio_child=None; self.state['audio'].update(state='本次播放器已停止',can_stop=False)
            self.save_audio()
            return {'stopped':True,'owner':'portal'}
        return {'stopped':False,'note':'门户未持有播放器；原服务短音自然结束'}
