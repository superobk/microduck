"""Host tooling tests. These DO NOT compile or execute the Rust controller."""
from __future__ import annotations
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import tomllib
import unittest
import shutil

ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
patch=load('headless_patch',ROOT/'apply.py')
config=load('headless_config',ROOT/'tools/prepare_config.py')
wire=load('headless_wire',ROOT/'tools/pty_bus_test.py')

class ApplicatorTests(unittest.TestCase):
    def test_payload_allowlist_ignores_finder_metadata_but_refuses_other_files(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);shutil.copytree(ROOT/'payload',root/'payload')
            (root/'payload/.DS_Store').write_bytes(b'finder metadata')
            self.assertEqual([r for r,_ in patch.payload_sources(root)],list(patch.PAYLOAD_FILES))
            (root/'payload/extra.rs').write_text('// undeclared Rust also refused')
            with self.assertRaises(RuntimeError):patch.payload_sources(root)
    def test_declared_payload_symlink_refused(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);shutil.copytree(ROOT/'payload',root/'payload')
            p=root/'payload'/patch.PAYLOAD_FILES[0];p.unlink();p.symlink_to(ROOT/'payload'/patch.PAYLOAD_FILES[0])
            with self.assertRaises(RuntimeError):patch.payload_sources(root)
    def test_git_blob_hash_agrees_with_git(self):
        data=b'one\nline\xff\x00'
        actual=subprocess.check_output(['git','hash-object','--stdin'],input=data).decode().strip()
        self.assertEqual(patch.blob_hash(data),actual)
    def test_anchor_count_rejects_no_match(self):
        with self.assertRaises(RuntimeError):patch.replace_checked('x',dict(name='test',old='y',new='z',count=1))
    def test_anchor_count_rejects_ambiguous_match(self):
        with self.assertRaises(RuntimeError):patch.replace_checked('xx',dict(name='test',old='x',new='y',count=1))
    def test_ordered_replacements(self):
        self.assertEqual(patch.transform(b'abc',[dict(name='a',old='a',new='d',count=1),dict(name='b',old='db',new='ef',count=1)]),b'efc')
    def test_every_declared_edit_has_a_working_isolated_replacement(self):
        # Isolated fixtures check applicator mechanics, NOT all anchors against upstream.
        manifest=json.loads((ROOT/'edits.json').read_text())
        for file in manifest['files'].values():
            for edit in file['edits']:
                self.assertTrue(edit['old']);self.assertGreater(edit['count'],0)
                text='\n--FIXTURE--\n'.join([edit['old']]*edit['count'])
                expected='\n--FIXTURE--\n'.join([edit['new']]*edit['count'])
                with self.subTest(name=edit['name']):self.assertEqual(patch.replace_checked(text,edit),expected)
    def test_no_network_or_policy_files_in_patch_targets(self):
        manifest=json.loads((ROOT/'edits.json').read_text())
        self.assertEqual(set(manifest['files']),{'duck-control/src/lib.rs','duck-control/src/bus.rs','robotd/src/control.rs','robotd/src/main.rs'})
        for p in (ROOT/'payload').rglob('*'):
            if p.is_file():self.assertEqual(p.suffix,'.rs')
    def test_path_traversal_and_absolute_path_refused(self):
        with tempfile.TemporaryDirectory() as t:
            repo=Path(t).resolve()
            for rel in ('../escape','/tmp/escape'):
                with self.assertRaises(RuntimeError):patch.safe_target(repo,rel)
    def test_symlink_refused(self):
        with tempfile.TemporaryDirectory() as t:
            repo=Path(t).resolve();(repo/'real').write_text('x');(repo/'link').symlink_to(repo/'real')
            with self.assertRaises(RuntimeError):patch.safe_target(repo,'link')
    def test_atomic_write_preserves_mode(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'f';p.write_bytes(b'old');p.chmod(0o640);patch.atomic_write(p,b'new')
            self.assertEqual(p.read_bytes(),b'new');self.assertEqual(p.stat().st_mode&0o777,0o640)
    def test_synthetic_repo_plan_idempotent_and_reverse(self):
        with tempfile.TemporaryDirectory() as t:
            repo=Path(t).resolve();subprocess.run(['git','init','-q',str(repo)],check=True)
            for k,v in [('user.name','Fixture'),('user.email','fixture@example.invalid')]:patch.git(repo,'config',k,v)
            p=repo/'source.rs';p.write_text('old\n');patch.git(repo,'add','.');patch.git(repo,'commit','-qm','fixture')
            commit=patch.git(repo,'rev-parse','HEAD').decode().strip()
            manifest={'upstream_commit':commit,'files':{'source.rs':{'git_blob_sha1':patch.blob_hash(b'old\n'),'edits':[dict(name='fixture',old='old',new='new',count=1)]}}}
            items=patch.plan(repo,manifest,False)
            for rel,_,new in items:
                if new is not None:patch.atomic_write(repo/rel,new)
            self.assertEqual(p.read_text(),'new\n')
            self.assertTrue(all(old==new for _,old,new in patch.plan(repo,manifest,False)))
            for rel,_,new in patch.plan(repo,manifest,True):
                if new is None:(repo/rel).unlink()
                else:patch.atomic_write(repo/rel,new)
            self.assertEqual(p.read_text(),'old\n')
            self.assertFalse((repo/'duck-control/src/morphology.rs').exists())
    def test_synthetic_repo_refuses_local_changes_before_any_write(self):
        with tempfile.TemporaryDirectory() as t:
            repo=Path(t).resolve();subprocess.run(['git','init','-q',str(repo)],check=True)
            patch.git(repo,'config','user.name','Fixture');patch.git(repo,'config','user.email','fixture@example.invalid')
            p=repo/'source.rs';p.write_text('old\n');patch.git(repo,'add','.');patch.git(repo,'commit','-qm','fixture')
            manifest={'upstream_commit':patch.git(repo,'rev-parse','HEAD').decode().strip(),'files':{'source.rs':{'git_blob_sha1':patch.blob_hash(b'old\n'),'edits':[dict(name='fixture',old='old',new='new',count=1)]}}}
            p.write_text('user edit\n')
            with self.assertRaises(RuntimeError):patch.plan(repo,manifest,False)
            self.assertEqual(p.read_text(),'user edit\n');self.assertFalse((repo/'duck-control').exists())

class ConfigurationTests(unittest.TestCase):
    def test_preserves_existing_policy_tuning_and_paths(self):
        before='''# keep me\n[bus]\nport="/dev/ttyS2"\n[policy]\nwalk="/old/walk.onnx"\nstand="/old/stand.onnx"\naction_scale=0.9\nstanding_action_scale=1.0\nstanding_gain_ratio=0.8\ngain=200\nhead_lowpass=0.5\nlegs_lowpass=0.7\n[audio]\nenabled=true\n'''
        after=config.prepare(before,None,False)
        p=tomllib.loads(after)
        for k,v in tomllib.loads(before)['policy'].items():self.assertEqual(p['policy'][k],v)
        self.assertTrue(p['audio']['enabled']);self.assertIn('# keep me',after)
        self.assertEqual(p['policy']['roulade'],'none')
    def test_missing_final_newline(self):
        result=config.prepare('[policy]\nwalk="a.onnx"',None,False)
        self.assertEqual(tomllib.loads(result)['policy']['walk'],'a.onnx')
    def test_empty_input(self):
        p=tomllib.loads(config.prepare('',None,False));self.assertEqual(p['control']['hz'],50)
    def test_defaults_do_not_invent_tuning(self):
        p=tomllib.loads(config.prepare('',None,False))['policy']
        for key in ('action_scale','gain','legs_lowpass','walk','stand'):self.assertNotIn(key,p)
    def test_regulated_rail_option(self):
        p=tomllib.loads(config.prepare('',None,True))
        self.assertFalse(p['policy']['voltage_adapt']);self.assertFalse(p['safety']['battery_empty_shutdown'])
    def test_plain_sync_option(self):
        p=tomllib.loads(config.prepare('[bus]\nfast_sync_read=true\n',None,False,True))
        self.assertFalse(p['bus']['fast_sync_read'])
    def test_walk_path_escaping(self):
        v='/tmp/model "original".onnx';p=tomllib.loads(config.prepare('',v,False));self.assertEqual(p['policy']['walk'],v)
    def test_invalid_toml_is_not_rewritten(self):
        with self.assertRaises(tomllib.TOMLDecodeError):config.prepare('[policy',None,False)
    def test_array_tables_are_preserved_not_confused_with_policy_section(self):
        before='[policy]\nwalk="a"\n[[policy.skill]]\nname="custom"\npath="custom.onnx"\n'
        p=tomllib.loads(config.prepare(before,None,False))
        self.assertEqual(p['policy']['skill'][0]['name'],'custom')
    def test_profiles_default_to_bench_and_mouth_is_independent(self):
        a=json.loads((ROOT/'profiles/headless-mouth.json').read_text());b=json.loads((ROOT/'profiles/legs10.json').read_text())
        self.assertTrue(a['mouth_present']);self.assertFalse(b['mouth_present'])
        self.assertFalse(a['allow_motion']);self.assertFalse(b['allow_motion'])
        self.assertEqual(a['locked_head_rad'],[.3491,.3491,0.,0.])
    def test_configuration_generator_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as t:
            src=Path(t)/'a.toml';dst=Path(t)/'b.toml';src.write_text('');dst.write_text('keep')
            p=subprocess.run([os.sys.executable,str(ROOT/'tools/prepare_config.py'),'--input',str(src),'--output',str(dst)],capture_output=True)
            self.assertNotEqual(p.returncode,0);self.assertEqual(dst.read_text(),'keep')

class ProtocolCodecTests(unittest.TestCase):
    def test_known_ping_packet_crc(self):
        self.assertEqual(wire.packet(1,1),bytes.fromhex('ff ff fd 00 01 03 00 01 19 4e'))
    def test_stuffing_round_trip(self):
        for b in (b'',b'\xff\xff\xfd',b'\xff\xff\xfd\xfd',b'abc\xff\xff\xfd'*4):
            self.assertEqual(wire.unstuff(wire.stuff(b)),b)
    def test_packet_length_covers_stuffed_bytes(self):
        p=wire.packet(1,3,b'\xff\xff\xfd')
        size=int.from_bytes(p[5:7],'little');self.assertEqual(size,len(p)-7)
        self.assertEqual(wire.crc16(p[:-2]),int.from_bytes(p[-2:],'little'))
    def test_mock_sync_read_preserves_requested_order(self):
        bus=wire.Bus(-1,[20,34,10]);sent=[];bus.send=lambda i,data=b'',err=0:sent.append(i)
        bus.handle(254,0x82,bytes([132,0,4,0,20,34,10]))
        self.assertEqual(sent,[20,34,10])
    def test_mock_missing_device_does_not_return_a_fake_zero(self):
        bus=wire.Bus(-1,[20,10]);sent=[];bus.send=lambda i,data=b'',err=0:sent.append(i)
        bus.read(30,132,4);bus.dropped.add(10);bus.read(10,132,4)
        self.assertEqual(sent,[])
    def test_mock_sync_write_has_no_missing_ids(self):
        bus=wire.Bus(-1,[20,10]);bus.handle(254,0x83,bytes([116,0,4,0,20,0,8,0,0,10,1,8,0,0]))
        self.assertEqual([e[1] for e in bus.events if e[0]=='write'],[20,10])

if __name__=='__main__':unittest.main(verbosity=2)
