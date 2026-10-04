"""Durability and migration-input checks; never contacts a remote machine."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from audit_command import run

class AuditTests(unittest.TestCase):
    def test_failed_child_retains_exit_output_and_meaning(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            code=run([sys.executable,'-c','print("audited failure");raise SystemExit(7)'],root,'验证失败仍保留证据','不要把失败写作成功')
            self.assertEqual(code,7)
            d=next(root.glob('*/operation.json')).parent;r=json.loads((d/'operation.json').read_text())
            self.assertEqual(r['exit_code'],7);self.assertEqual(r['status'],'failed_or_interrupted')
            self.assertIn('audited failure',(d/'output.log').read_text())
            self.assertEqual(r['output_sha256'],hashlib.sha256((d/'output.log').read_bytes()).hexdigest())
            self.assertEqual(r['meaning'],'验证失败仍保留证据')
            ledger=(root/'COMMANDS_INCREMENTAL.md').read_text()
            self.assertIn('`7`',ledger);self.assertIn(str(d/'operation.json'),ledger)
    def test_nonexistent_command_still_has_record(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.assertEqual(run(['/nonexistent/zero3w-command'],root,'不存在的命令','保留错误'),127)
            r=json.loads(next(root.glob('*/operation.json')).read_text());self.assertEqual(r['exit_code'],127)

class DeployTests(unittest.TestCase):
    def invoke(self,root,*args):
        return subprocess.run([sys.executable,str(Path(__file__).with_name('deploy.py')),*args,'--target','root@zero3w.invalid','--audit-root',str(root)],capture_output=True,text=True)
    def test_mutation_requires_power_isolation_and_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for action in ['backup','install','rollback']:
                r=self.invoke(root,action,'--run-id','fixture','--plan-only')
                self.assertNotEqual(r.returncode,0);self.assertFalse(list(root.iterdir()))
    def test_run_id_injection_is_refused_before_any_upload(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);r=self.invoke(root,'prepare','--run-id','../bad;id','--plan-only')
            self.assertNotEqual(r.returncode,0);self.assertFalse(list(root.iterdir()))
    def test_plan_only_archives_exact_script_without_remote_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);r=self.invoke(root,'preflight','--run-id','fixture','--plan-only')
            self.assertEqual(r.returncode,0,r.stderr)
            plan=json.loads(r.stdout);self.assertFalse(plan['remote_executed'])
            artifact=next(root.glob('scripts/*/remote_candidate.py'))
            self.assertEqual(plan['script_sha256'],hashlib.sha256(artifact.read_bytes()).hexdigest())
            self.assertFalse(list(root.glob('*/operation.json')))
if __name__=='__main__':unittest.main(verbosity=2)
