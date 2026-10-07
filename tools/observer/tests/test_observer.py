"""Failure paths matter more than a green dashboard. No real service/device calls."""
import io
import json
import math
import os
from pathlib import Path
import pty
import socket
import struct
import sys
import tarfile
import tempfile
import threading
import time
import tty
import unittest
from urllib.request import Request,build_opener,ProxyHandler
from urllib.error import HTTPError
from unittest.mock import patch

HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
import protocol as p
import runtime
from runtime import Engine
from server import dispatch,request_local,LocalServer,HelperHandler,ThreadingHTTPServer,Handler,EventFrames
from install import check_archive,verify_release

# Loopback contract tests must bypass workstation network proxy discovery.
urlopen=build_opener(ProxyHandler({})).open

class ProtocolTests(unittest.TestCase):
    def test_crc_and_stuffing(self):
        packet=p.encode(20,0x55,b'\0\xff\xff\xfdhello')
        self.assertEqual(p.decode(packet),(20,0x55,b'\0\xff\xff\xfdhello'))
        bad=bytearray(packet);bad[-1]^=1
        with self.assertRaises(ValueError):p.decode(bad)

    def test_only_expected_ids_and_read_registers(self):
        for sid in p.SERVO_IDS:
            ident,inst,body=p.decode(p.read_request(sid,64,7));self.assertEqual(inst,2);self.assertEqual(ident,sid)
        for args in [(30,64,7),(20,116,4),(200,256,96),(20,64,1),(1,0,3)]:
            with self.assertRaises(ValueError):p.read_request(*args)
        _,instruction,params=p.decode(p.sync_request());self.assertEqual(instruction,0x82)
        self.assertEqual(list(params[4:]),p.BUS_IDS)
        for sid in p.BUS_IDS:
            ident,instruction,body=p.decode(p.identity_request(sid))
            self.assertEqual((ident,instruction,body),(sid,1,b''))
        with self.assertRaises(ValueError):p.identity_request(30)
        # This literal is the original Rust controller's IMU-first wire contract,
        # rather than deriving the expected result from the implementation list.
        _,_,data=p.decode(p.sync_request());self.assertEqual(list(data[4:]),[200,20,21,22,23,24,34,10,11,12,13,14])
        _,_,data=p.decode(p.sync_request(ids=[200,10]));self.assertEqual(list(data[4:]),[200,10])
        with self.assertRaises(ValueError):p.sync_request(ids=[10,200])
        for ids in [[],[30],[10,10],[1]]:
            with self.assertRaises(ValueError):p.sync_request(ids=ids)
        with self.assertRaises(ValueError):p.sync_request(True,[200])

    def test_servo_units_and_right_slots(self):
        data=p.servo_decode(10,struct.pack('<hhii',0,-42,60,2048),128)
        self.assertEqual(data['slot'],10);self.assertEqual(data['current_ma'],42);self.assertEqual(data['position'],0)
        self.assertAlmostEqual(data['velocity'],.229*math.tau)
        self.assertEqual(data['status_byte'],128)  # accepted voltage alert remains raw

    def test_nonfinite_uninitialised_and_norm(self):
        for raw in [bytes(12),struct.pack('<hhh',0,0,0)+struct.pack('<eee',float('nan'),0,0),
                    struct.pack('<hhh',0,0,0)+struct.pack('<eee',1,1,0)]:
            with self.assertRaises(ValueError):p.imu_decode(raw)
        for q in [[0,0,0,0],[1,float('nan'),0,0]]:
            with self.assertRaises(ValueError):p.qnorm(q)

    def test_default_mount(self):
        raw=struct.pack('<hhh',100,200,300)+struct.pack('<eee',0,math.sqrt(.5),0)
        imu=p.imu_decode(raw)
        self.assertAlmostEqual(imu['trunk_quat'][0],1,places=6)
        self.assertAlmostEqual(imu['gyro'][0],300*.0175*math.pi/180)
        self.assertAlmostEqual(imu['gyro'][2],-100*.0175*math.pi/180)

    def test_rust_golden(self):
        fixture=HERE/'tests/golden.json'
        self.assertTrue(fixture.exists(),'generate Rust golden before claiming decoder equivalence')
        expected=json.loads(fixture.read_text());imu=p.imu_decode(bytes.fromhex(expected['raw_hex']))
        for key,source in [('gyro','gyro'),('trunk_quat','quat'),('gravity','gravity')]:
            for a,b in zip(imu[key],expected[source]):self.assertAlmostEqual(a,b,places=12)

    def test_ordinary_linux_pty_contract(self):
        # Exercise actual framed IO on a PTY. Servo alerts must not be confused with
        # protocol errors, and no write/enable instruction may leave the transport.
        master,slave=pty.openpty();tty.setraw(slave)
        port=p.ReadOnlyPort.__new__(p.ReadOnlyPort);port.fd=slave;port.old=None
        observed=[]
        def device():
            request=os.read(master,4096);ident,inst,params=p.decode(request);observed.append(inst)
            self.assertEqual((ident,inst),(254,0x82));self.assertEqual(list(params[4:]),p.BUS_IDS)
            for sid in p.BUS_IDS:
                raw=(struct.pack('<hhh',0,0,0)+struct.pack('<eee',.25,.25,0)) if sid==200 else struct.pack('<hhii',0,10,0,2048)
                os.write(master,p.encode(sid,0x55,bytes([128 if sid!=200 else 0])+raw))
        worker=threading.Thread(target=device);worker.start()
        try:
            rows=port.sync();self.assertEqual([r[0] for r in rows],p.BUS_IDS);self.assertEqual(len(rows),12)
            self.assertEqual(observed,[0x82])
        finally:worker.join(timeout=1);os.close(master);os.close(slave);port.fd=None

    def test_fast_full_chain_and_truncation(self):
        body=b'\x55'+b''.join(bytes([0,sid])+bytes(12)+bytes(2) for sid in p.BUS_IDS)
        data=p.HEADER+b'\xfe'+struct.pack('<H',len(body))+body
        data=data[:-2]+struct.pack('<H',p.crc16(data[:-2]))
        self.assertEqual([r[0] for r in p.parse_fast(data)],p.BUS_IDS)
        with self.assertRaises(ValueError):p.parse_fast(data[:88])

class EngineTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.engine=Engine(self.tmp.name,config='/nonexistent',offline=True)
    def tearDown(self):
        if self.engine.record_file:self.engine.record_file.close()
        self.tmp.cleanup()

    def imu_samples(self):
        value=p.imu_decode(struct.pack('<hhh',0,0,0)+struct.pack('<eee',0,math.sqrt(.5),0))
        for _ in range(25):self.engine.update('imu',value)

    def test_missing_not_zero_and_stale(self):
        snap=self.engine.snapshot();self.assertEqual(snap['imu']['status'],'未验证');self.assertNotIn('trunk_quat',snap['imu'])
        self.imu_samples();self.engine.state['imu']['received']=time.monotonic()-2
        self.assertEqual(self.engine.snapshot()['imu']['status'],'过期')

    def test_mount_not_applied_twice(self):
        self.imu_samples();self.assertAlmostEqual(self.engine.snapshot()['imu']['trunk_quat'][0],1,places=6)

    def test_calibration_and_undo(self):
        self.imu_samples();self.engine.perform('a'*32,'calibrate',{'mode':'reference'})
        self.assertEqual(self.engine.profile['revision'],'a'*32)
        self.engine.perform('b'*32,'calibrate',{'mode':'undo'});self.assertEqual(self.engine.profile['reference'],[1,0,0,0])
        self.assertEqual(self.engine.profile['scope'],'仅门户观测，不修改控制器或固件')

    def test_calibration_rejects_stale(self):
        with self.assertRaises(RuntimeError):self.engine.calibrate('a'*32,{'mode':'yaw'})
        self.assertFalse((self.engine.root/'profile.json').exists())

    def test_service_and_rpc_allowlist(self):
        for unit in ['robotd','updaterd','zero3w-11servo-candidate','mediad.service; reboot']:
            with self.assertRaises(ValueError):self.engine.change_service('a'*32,unit,'start')
        with self.assertRaises(ValueError):runtime.rpc('/fake','robot.init')
        with self.assertRaises(ValueError):dispatch(self.engine,'exec',{'cmd':'true'})

    def test_audit_failure_prevents_mutation(self):
        with patch.object(self.engine,'journal',side_effect=RuntimeError('disk full')):
            with self.assertRaises(RuntimeError):self.engine.submit('observe',{'enabled':True})
        self.assertFalse(self.engine.bus_requested)

    def test_queue_bounded(self):
        for _ in range(8):self.engine.submit('observe',{'enabled':False})
        with self.assertRaises(RuntimeError):self.engine.submit('observe',{'enabled':True})

    def test_record_capacity_does_not_stop_observation(self):
        self.engine.recording=True;self.engine.record_bytes=self.engine.record_cap
        self.engine.write_record({'utc':runtime.utc(),'component':'imu','values':{}})
        self.assertFalse(self.engine.recording);self.assertTrue(self.engine.observing)

    def test_no_sound_success_without_test(self):
        with self.assertRaises(RuntimeError):self.engine.perform('a'*32,'audio_heard',{'heard':True})
        self.assertEqual(self.engine.stop_audio()['stopped'],False)

    def test_audio_levels_closed(self):
        for level in [-1,0,1,float('nan'),True,'0.1']:
            with self.assertRaises(ValueError):self.engine.play_audio({'level':level})

    def test_restart_boot_intent(self):
        self.engine.perform('a'*32,'observe',{'enabled':True})
        restored=Engine(self.tmp.name,config='/nonexistent',offline=True)
        self.assertTrue(restored.bus_requested)
        with patch('runtime.boot_id',return_value='different-boot'):
            rebooted=Engine(self.tmp.name,config='/nonexistent',offline=True)
        self.assertFalse(rebooted.bus_requested)

    def test_export_contains_full_audit_and_csv(self):
        result=self.engine.perform('a'*32,'export',{})
        self.assertEqual(result['download'],'/api/download/'+'a'*32+'.zip')
        import zipfile
        with zipfile.ZipFile(self.engine.root/'exports'/('a'*32+'.zip')) as z:
            self.assertIn('summary.csv',z.namelist());self.assertTrue(any(x.startswith('audit/') for x in z.namelist()))

    def test_restore_detects_pid_drift(self):
        tid='a'*32
        tx={'unit':'tofd','boot':self.engine.boot,'before':{'ActiveState':'active'},'after':{'ActiveState':'active','MainPID':'12','UnitFileState':'enabled','fingerprints':{}}}
        (self.engine.root/'transactions'/(tid+'.json')).write_text(json.dumps(tx))
        with patch('runtime.service_state',return_value={**tx['after'],'MainPID':'13'}):
            with self.assertRaises(RuntimeError):self.engine.perform('b'*32,'restore',{'transaction':tid})

    def test_indirect_updater_activation_refused(self):
        states={'tofd':{'query_ok':True,'ActiveState':'inactive','Wants':'relay.service'},
                'updaterd':{'query_ok':True,'ActiveState':'inactive'}}
        with patch('runtime.service_state',return_value={'query_ok':True,'ActiveState':'inactive','Wants':'updaterd.service'}):
            with self.assertRaisesRegex(RuntimeError,'updaterd'):self.engine.guard_dependencies('tofd',states)

    def test_moving_pose_cannot_save_reference(self):
        self.imu_samples()
        now=time.monotonic()
        self.engine.samples.append((now,{**self.engine.samples[-1][1],'trunk_quat':[math.sqrt(.5),0,0,math.sqrt(.5)]}))
        with self.assertRaises(RuntimeError):self.engine.calibrate('a'*32,{'mode':'reference'})
        self.assertFalse((self.engine.root/'profile.json').exists())

    def test_completed_player_still_needs_hearing_confirmation(self):
        from unittest.mock import Mock
        child=Mock(pid=999999,returncode=0);child.poll.return_value=0
        self.engine.audio_child=child;self.engine.state['audio'].update(state='播放中',can_stop=True)
        self.engine.inspect_audio()
        self.assertEqual(self.engine.state['audio']['state'],'播放器正常结束')
        self.assertIsNone(self.engine.state['audio']['heard']);self.assertFalse(self.engine.state['audio']['can_stop'])

    def test_audio_confirmation_survives_own_restart(self):
        self.engine.state['audio'].update(test_id='a'*32,state='播放器正常结束',exit_code=0)
        self.engine.perform('b'*32,'audio_heard',{'heard':True})
        restored=Engine(self.tmp.name,config='/nonexistent',offline=True)
        self.assertTrue(restored.state['audio']['heard']);self.assertEqual(restored.state['audio']['exit_code'],0)

    def test_absent_servos_do_not_become_connected(self):
        self.engine.state['servo']['devices']={'20':{'id':20,'connected':False}}
        snap=self.engine.snapshot()
        self.assertEqual(snap['servo']['status'],'离线');self.assertEqual(snap['servo']['devices']['20']['status'],'离线')
        self.assertNotIn('position',snap['servo']['devices']['20'])

    def test_root_mount_is_passed_after_option_separator(self):
        from unittest.mock import Mock
        with patch('runtime.subprocess.run',return_value=Mock(returncode=0,stdout='ActiveState=active\n')) as command:
            self.assertTrue(runtime.service_state('-.mount')['query_ok'])
        self.assertEqual(command.call_args.args[0][-2:],['--','-.mount'])

    def test_report_stays_bounded_after_long_observation(self):
        for i in range(3600):self.engine.summary.append({'i':i})
        result=dispatch(self.engine,'report',{})
        self.assertEqual(len(result['summary']),60);self.assertEqual(result['summary'][-1]['i'],3599)
        with self.assertRaises(ValueError):dispatch(self.engine,'report',{'limit':3600})

    def test_stale_or_other_owner_never_allows_uart(self):
        self.assertFalse(self.engine.ownership_ready())
        self.engine.owner_checked=time.monotonic();self.assertTrue(self.engine.ownership_ready())
        self.engine.owner_pids=[123];self.assertFalse(self.engine.ownership_ready())
        self.engine.owner_pids=[];self.engine.owner_checked=time.monotonic()-1;self.assertFalse(self.engine.ownership_ready())

    def test_three_bad_frames_latch_without_reopening_or_resetting_streak(self):
        from unittest.mock import Mock
        port=Mock()
        port.identity.side_effect=lambda ident:((1030 if ident==200 else 1200).to_bytes(2,'little')+bytes([3 if ident==200 else 52]),0)
        port.read.return_value=(bytes(7),0)
        port.sync.side_effect=TimeoutError('missing response')
        self.engine.offline=False;self.engine.observing=True;self.engine.bus_requested=True
        with patch('runtime.ReadOnlyPort',return_value=port) as opener,patch.object(self.engine,'controllers_stopped',return_value=True),patch.object(self.engine,'ownership_ready',return_value=True):
            thread=threading.Thread(target=self.engine.bus_loop);thread.start()
            deadline=time.monotonic()+2
            while self.engine.bus_requested and time.monotonic()<deadline:time.sleep(.01)
            self.engine.shutdown.set();thread.join(2)
        self.assertFalse(self.engine.bus_requested);self.assertEqual(port.sync.call_count,3)
        self.assertEqual(opener.call_count,1);port.close.assert_called_once()
        self.assertEqual(self.engine.state['servo']['errors'],3)

    def test_unknown_stale_or_inactive_controller_is_not_running(self):
        self.assertFalse(self.engine.controller_running())
        self.engine.state['service_checked_at']=time.monotonic()
        self.engine.state['services']={'robotd':{'query_ok':True,'ActiveState':'inactive','MainPID':'0'}}
        self.assertFalse(self.engine.controller_running())
        self.engine.state['services']['robotd'].update(ActiveState='active',MainPID='123')
        self.assertTrue(self.engine.controller_running())
        self.engine.state['service_checked_at']=time.monotonic()-3
        self.assertFalse(self.engine.controller_running())

class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
    def tearDown(self):self.server.shutdown();self.server.server_close();self.thread.join()

    def test_write_requires_same_origin_and_closed_action(self):
        req=Request(self.url+'/api/action',data=b'{}',headers={'Origin':'http://elsewhere','X-Observer':'1'},method='POST')
        with self.assertRaises(HTTPError) as raised:urlopen(req,timeout=3)
        self.assertEqual(raised.exception.code,403)
        raised.exception.close()

    def test_static_traversal_and_arbitrary_rpc_are_not_routes(self):
        for path in ['/../../runtime.py','/api/robot.init']:
            with self.assertRaises(HTTPError) as raised:urlopen(self.url+path,timeout=3)
            self.assertEqual(raised.exception.code,404)
            raised.exception.close()

class EventFrameTests(unittest.TestCase):
    def test_multiple_pages_share_sample_without_refreshing_its_age(self):
        calls=[]
        def provider():
            calls.append(1);time.sleep(.01)
            return {'imu':{'received':123.,'age_ms':7.},'version':{'version':'test'}}
        frames=EventFrames(provider,interval=.1)
        results=[]
        threads=[threading.Thread(target=lambda:results.append(frames.get())) for _ in range(12)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        self.assertEqual(len(calls),1);self.assertEqual(len(set(results)),1)
        row=json.loads(results[0].decode().removeprefix('data: '))
        self.assertEqual(row['imu']['received'],123.)
        self.assertEqual(row['imu']['age_ms'],7.)
        time.sleep(.11);frames.get();self.assertEqual(len(calls),2)

    def test_helper_outage_does_not_reuse_expired_frame(self):
        from unittest.mock import Mock
        provider=Mock(side_effect=[{'imu':{'status':'实时'}},RuntimeError('helper down')])
        frames=EventFrames(provider,interval=.01);frames.get();time.sleep(.02)
        with self.assertRaisesRegex(RuntimeError,'helper down'):frames.get()

class AcceptanceTests(unittest.TestCase):
    def test_version_reboot_and_helper_restart_do_not_form_continuous_pass(self):
        from acceptance import continuity
        first={'boot':'old','version':{'version':'r7'},'uptime_s':100}
        self.assertIsNone(continuity(first,{**first,'uptime_s':200}))
        for change in [{'boot':'new'},{'version':{'version':'r8'}},{'uptime_s':1}]:
            self.assertIsNotNone(continuity(first,{**first,**change}))

class ArchiveTests(unittest.TestCase):
    def test_traversal_and_links_rejected(self):
        for kind in ['traversal','symlink']:
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'bundle.tar.gz'
                with tarfile.open(path,'w:gz') as tar:
                    item=tarfile.TarInfo('../escape' if kind=='traversal' else 'link')
                    if kind=='symlink':item.type=tarfile.SYMTYPE;item.linkname='/etc/robot/robotd.toml'
                    tar.addfile(item,io.BytesIO(b''))
                from install import digest
                with self.assertRaises(RuntimeError):check_archive(path,digest(path))

    def test_manifest_tamper(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'server.py').write_text('safe')
            (path/'MANIFEST.json').write_text(json.dumps({'files':{'server.py':'0'*64}}))
            with self.assertRaises(RuntimeError):verify_release(path)

if __name__=='__main__':unittest.main()
