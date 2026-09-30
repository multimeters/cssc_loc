"""Exercise the public mode dispatcher without starting ROS or opening a bag."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

from aps_bag_localization.configuration import load_config

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from runtime_support import save_configuration


class RuntimeEntryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.config = load_config(ROOT / 'config/localization.yaml')
        self.config['paths']['bag'] = None
        self.config['paths']['map'] = str(self.directory / 'map.pcd')
        self.config['paths']['map_metadata'] = None
        self.config['paths']['output_root'] = str(self.directory / 'outputs')
        Path(self.config['paths']['map']).write_text(
            'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n', encoding='utf8')
        self.path = self.directory / 'localization.yaml'

    def invoke(self, *args):
        self.path.write_text(yaml.safe_dump(self.config), encoding='utf8')
        return subprocess.run(
            [sys.executable, str(ROOT / 'scripts/run_localization.py'),
             '--config', str(self.path), '--check', *args],
            capture_output=True, text=True, encoding='utf8', timeout=15)

    def test_live_check_does_not_require_or_open_bag(self):
        for bag in (None, str(self.directory / 'missing-bag')):
            self.config['paths']['bag'] = bag
            result = self.invoke()
            self.assertEqual(result.returncode, 0, result.stderr)
            result = json.loads(result.stdout)
            self.assertEqual(result['模式'], 'live')
            self.assertTrue(result['系统时钟'])
            self.assertFalse(result['需要录包'])
        self.assertFalse(Path(self.config['paths']['output_root']).exists())

    def test_replay_override_requires_bag_instead_of_starting_live(self):
        result = self.invoke('--mode', 'replay')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('回放模式需要 paths.bag', result.stderr)

    def test_live_rejects_replay_only_options(self):
        result = self.invoke('--mode', 'live', '--max-bag-seconds', '30')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('unrecognized arguments', result.stderr)

    def test_snapshot_is_self_contained_and_keeps_resolved_clock_mode(self):
        self.path.write_text(yaml.safe_dump(self.config), encoding='utf8')
        for mode, sim_time in (('live', False), ('replay', True)):
            effective = load_config(self.path, mode=mode)
            snapshot = save_configuration(effective, self.directory / mode)
            restored = load_config(snapshot)
            self.assertEqual(restored['runtime'], {'mode': mode, 'use_sim_time': sim_time})
            self.assertEqual(effective['_native_parameters'], restored['_native_parameters'])
            for name, target in restored['native_parameters'].items():
                self.assertTrue(Path(target).is_relative_to(self.directory / mode))
                self.assertEqual(Path(target).read_bytes(),
                                 Path(effective['native_parameters'][name]).read_bytes())


if __name__ == '__main__':
    unittest.main()
