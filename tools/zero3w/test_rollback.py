"""Failure-path checks in private temporary directories; no root/systemd/SSH.

Exercise interruption at each subset of unit writes, repeated rollback, later
full mechanical restoration, and drift/symlink/forged ownership refusal. These
verify recovery boundaries, not the existence of real board components.
"""
import contextlib
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import remote_candidate as remote


class RollbackTests(unittest.TestCase):
    def setup_case(self, tmp):
        root = Path(tmp) / 'candidate'; root.mkdir()
        holds = {Path(tmp)/'robotd.d/hold.conf': remote.HOLD,
                 Path(tmp)/'updaterd.d/hold.conf': remote.UPDATE_HOLD}
        service = Path(tmp)/'candidate.service'
        contents = {**holds, service: remote.candidate_body(root)}
        expected = {str(p): hashlib.sha256(text.encode()).hexdigest() for p, text in contents.items()}
        (root/'transaction.json').write_text(json.dumps({'owned_systemd_files': expected}))
        calls = []
        def systemctl(argv, check=True):
            calls.append(argv)
            return subprocess.CompletedProcess(argv, 0, 'loaded\n' if 'show' in argv else '', '')
        return root, holds, service, contents, calls, systemctl

    def exercise(self, root, holds, service, systemctl, full=False):
        with patch.object(remote, 'OWNED', holds), patch.object(remote, 'CANDIDATE_SERVICE', service), \
             patch.object(remote, 'command', systemctl), contextlib.redirect_stdout(io.StringIO()):
            remote.rollback(root, full)

    def test_all_interrupted_unit_subsets_and_repeat_are_recoverable(self):
        # The completed manifest deliberately does not exist: interruption can
        # happen after either hold, before/after the candidate unit, or earlier.
        for subset in itertools.product((False, True), repeat=3):
            with self.subTest(subset=subset), tempfile.TemporaryDirectory() as tmp:
                root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
                for present, (p, text) in zip(subset, contents.items()):
                    if present:
                        p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
                self.exercise(root, holds, service, systemctl)
                self.exercise(root, holds, service, systemctl)
                self.assertFalse(service.exists())
                for present, p in zip(subset[:2], holds):
                    self.assertEqual(p.exists(), present)
                self.assertEqual(len(list(root.glob('rollback-*.json'))), 2)
                self.assertFalse(any('start' in argv or 'restart' in argv for argv in calls))

    def test_later_full_restoration_removes_holds_after_prior_partial_rollback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
            for p, text in contents.items():
                p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
            self.exercise(root, holds, service, systemctl)
            self.exercise(root, holds, service, systemctl, True)
            self.exercise(root, holds, service, systemctl, True)
            self.assertFalse(any(p.exists() for p in contents))
            self.assertFalse(any('start' in argv or 'restart' in argv for argv in calls))

    def test_drift_refuses_before_any_service_mutation_or_file_removal(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
            for p, text in contents.items():
                p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
            service.write_text('changed by another operator')
            with self.assertRaisesRegex(RuntimeError, 'changed since transaction'):
                self.exercise(root, holds, service, systemctl)
            self.assertEqual(calls, []); self.assertTrue(all(p.exists() for p in contents))

    def test_symlink_cannot_grant_ownership_even_with_matching_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
            outside = Path(tmp)/'somebody-elses-unit'; outside.write_text(contents[service]); service.symlink_to(outside)
            with self.assertRaisesRegex(RuntimeError, 'changed since transaction'):
                self.exercise(root, holds, service, systemctl)
            self.assertEqual(calls, []); self.assertTrue(outside.exists())

    def test_forged_manifest_cannot_select_unrelated_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
            outside = Path(tmp)/'official-file'; outside.write_text('preserve')
            (root/'transaction.json').write_text(json.dumps({'owned_systemd_files': {str(outside): remote.sha(outside)}}))
            with self.assertRaisesRegex(RuntimeError, 'ownership manifest'):
                self.exercise(root, holds, service, systemctl)
            self.assertEqual(calls, []); self.assertEqual(outside.read_text(), 'preserve')

    def test_prepare_only_has_nothing_to_undo(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, holds, service, contents, calls, systemctl = self.setup_case(tmp)
            (root/'transaction.json').unlink()
            self.exercise(root, holds, service, systemctl)
            self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
