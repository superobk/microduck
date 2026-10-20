"""Exercise the Mac command path with local SSH/SCP stubs; never connect to hardware."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


STUB = """#!/usr/bin/env python3
import io,json,os,pathlib,re,sys,tarfile,hashlib
args=sys.argv[1:]
with open(os.environ["CALLS"],"a") as f:
    f.write(json.dumps([pathlib.Path(sys.argv[0]).name,args])+"\\n")
if pathlib.Path(sys.argv[0]).name=="scp" and args[-1].startswith("/"):
    target=pathlib.Path(args[-1])
    name=re.search(r"/var/tmp/([^/]+)\\.tar\\.gz",args[0])[1]
    result=b'{"verdict":"FAIL"}\\n'
    sums=(hashlib.sha256(result).hexdigest()+"  result.json\\n").encode()
    archive=target/(name+".tar.gz")
    with tarfile.open(archive,"w:gz") as t:
        for filename,content in [("result.json",result),("SHA256SUMS",sums)]:
            info=tarfile.TarInfo(name+"/"+filename);info.size=len(content)
            t.addfile(info,io.BytesIO(content))
    (target/(name+".tar.gz.sha256")).write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+"  "+archive.name+"\\n")
if pathlib.Path(sys.argv[0]).name=="ssh" and "accept.py --stage" in args[-1]:
    sys.exit(2)
"""


class RunnerTests(unittest.TestCase):
    def test_stand_refreshes_helpers_without_install_and_fetches_failed_run(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            package, binaries = root / "package", root / "bin"
            package.mkdir()
            binaries.mkdir()
            for name in ("accept.py", "capture.py", "install-board.sh", "rollback-board.sh",
                         "99-imu-acceptance.conf", "manifest.json", "README.zh-CN.md",
                         "robotd-linux-aarch64"):
                (package / name).write_text("fixture\n")
            (package / "SHA256SUMS").write_text("".join(
                hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n"
                for p in sorted(package.iterdir())))
            for name in ("ssh", "scp"):
                stub = binaries / name
                stub.write_text(STUB)
                stub.chmod(0o755)
            env = dict(os.environ, PATH=str(binaries) + ":" + os.environ["PATH"],
                       IMU_ACCEPT_PACKAGE=str(package), IMU_ACCEPT_OUTPUT=str(root / "output"),
                       CALLS=str(root / "calls"), LC_ALL="invalid_TEST.UTF-8", LANG="en_MO.UTF-8")
            runner = str(Path(__file__).with_name("run.sh"))
            check = subprocess.run(["bash", runner, "--check-local"], env=env, capture_output=True, text=True)
            self.assertEqual(check.returncode, 0, check.stderr)
            self.assertFalse((root / "calls").exists())
            result = subprocess.run(["bash", runner, "--stage", "stand"], input="YES\n",
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            calls = [json.loads(line) for line in (root / "calls").read_text().splitlines()]
            uploads = [args for tool, args in calls if tool == "scp" and args[-1].startswith("root@")]
            self.assertEqual(len(uploads), 1)
            self.assertIn(str(package / "accept.py"), uploads[0])
            ssh_commands = [args[-1] for tool, args in calls if tool == "ssh"]
            self.assertTrue(any("python3 -X utf8 accept.py --stage stand" in c for c in ssh_commands))
            self.assertFalse(any("bash install-board.sh" in c for c in ssh_commands))
            self.assertNotIn("perl: warning", result.stderr)
            self.assertEqual(len(list((root / "output").glob("*/*/result.json"))), 1)


if __name__ == "__main__":
    unittest.main()
