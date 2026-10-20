import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import package


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PackageTests(unittest.TestCase):
    def test_test_only_changes_are_recorded_but_production_changes_rejected(self):
        for change in ("none", "test", "production"):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as d:
                root = Path(d)
                repo, source, output = root / "repo", root / "source", root / "output"
                here = repo / "scripts/imu-acceptance"
                here.mkdir(parents=True)
                source.mkdir()
                names = ["duck-control/src/a.rs", "robotd/src/main.rs", "Cargo.lock",
                         "duck-control/Cargo.toml", "robotd/Cargo.toml",
                         "duck-control/tests/a.rs", "docs/robot/imu-acceptance.md"]
                for name in names:
                    path = repo / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(name)
                for name in ("accept.py", "capture.py", "install-board.sh", "rollback-board.sh",
                             "99-imu-acceptance.conf"):
                    (here / name).write_text("test fixture")
                (source / "robotd-linux-aarch64").write_bytes(b"test fixture, not a robot binary")
                production = names[:5]
                source_hash = hashlib.sha256(b"".join(
                    name.encode() + b"\0" + (repo / name).read_bytes() for name in production)).hexdigest()
                # The all-source check must catch a production file absent from the
                # old audit's changed-file list too.
                manifest = dict(binary_sha256=sha(source / "robotd-linux-aarch64"),
                                source_hash=source_hash,
                                source_files_sha256={"duck-control/tests/a.rs": sha(repo / names[5])})
                (source / "manifest.json").write_text(json.dumps(manifest))
                if change == "test":
                    (repo / names[5]).write_text("test-only lint correction")
                elif change == "production":
                    (repo / names[0]).write_text("changed controller")
                with patch.object(package, "HERE", here), patch.object(package, "SHA", manifest["binary_sha256"]):
                    if change == "production":
                        with self.assertRaisesRegex(AssertionError, "production source differs"):
                            package.build(source, output)
                    else:
                        package.build(source, output)
                        built = json.loads((output / "manifest.json").read_text())
                        self.assertEqual(bool(built["packaging_test_only_changes"]), change == "test")
                        self.assertEqual(built["source_files_sha256"], manifest["source_files_sha256"])
                        self.assertEqual(sha(output / "robotd-linux-aarch64"), manifest["binary_sha256"])


if __name__ == "__main__":
    unittest.main()
