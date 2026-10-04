import copy
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_log as e


def frame(t=0):
    return {'t': t, 'safety': {'gravity': [0.,0.,-1.], 'fallen': False},
            'joints': [0.]*15, 'targets': [0.]*15,
            'currents_ma':[20.]*15,
            'control_loop': {'hz':50., 'missed':0},
            'movement': {'applied':[0.,0.,0.], 'limited_by':[]}, 'policy':'walk'}

class Tests(unittest.TestCase):
    def summary(self, fs): return e.summarize([json.dumps(x) for x in fs],0)
    def test_bare_state(self):
        s=self.summary([frame(0),frame(.04)])
        self.assertEqual(s['tilt_from_gravity_deg']['max'],0)
    def test_notification(self):
        s=self.summary([{'method':'robot.state','params':frame()}])
        self.assertEqual(s['input']['analyzed_frames'],1)
    def test_ignore_ack(self):
        s=self.summary([{'id':1,'result':{}},frame()])
        self.assertEqual(s['input']['ignored_nonstate_lines'],1)
    def test_no_frames(self):
        with self.assertRaises(ValueError):self.summary([{'id':1}])
    def test_restart(self):
        with self.assertRaises(ValueError):self.summary([frame(10),frame(1)])
    def test_skip(self):
        s=e.summarize([json.dumps(frame(t)) for t in (0,4,6,7)],5)
        self.assertEqual(s['input']['analyzed_frames'],2)
    def test_missing_not_zero(self):
        f=frame();del f['currents_ma'];del f['control_loop'];del f['safety']['fallen']
        s=self.summary([f])
        self.assertIsNone(s['daemon_control_loop_hz'])
        self.assertIsNone(s['fallen_true_frames'])
        self.assertIsNone(s['leg_reported_current_abs_ma']['left_knee'])
    def test_excludes_virtual_head(self):
        f=frame();f['targets'][5]=100;f['targets'][9]=100
        s=self.summary([f]);self.assertEqual(s['leg_target_minus_measured_abs_rad']['right_hip_yaw']['max'],0)
    def test_correct_right_leg_mapping(self):
        f=frame();f['targets'][10]=.1
        s=self.summary([f]);self.assertAlmostEqual(s['leg_target_minus_measured_abs_rad']['right_hip_yaw']['max'],.1)
    def test_nonfinite_rejected(self):
        f=frame();f['safety']['gravity'][0]=float('nan')
        s=self.summary([f]);self.assertIsNone(s['tilt_from_gravity_deg']);self.assertEqual(s['invalid_gravity_frames'],1)
    def test_subscriber_rate_not_loop_rate(self):
        s=self.summary([frame(t) for t in (0,.04,.08)])
        self.assertAlmostEqual(s['publication_hz_approx'],25)
        self.assertEqual(s['daemon_control_loop_hz']['mean'],50)
    def test_counter_delta(self):
        f,g=frame(0),frame(1);f['control_loop']['missed']=5;g['control_loop']['missed']=7
        self.assertEqual(self.summary([f,g])['loop_missed_counter_delta'],2)
    def test_fallen_edges(self):
        fs=[frame(t) for t in range(4)]
        fs[1]['safety']['fallen']=True;fs[2]['safety']['fallen']=True
        s=self.summary(fs);self.assertEqual(s['fallen_true_frames'],2);self.assertEqual(s['fallen_observed_rising_edges'],1)
    def test_bad_lines_counted(self):
        s=e.summarize(['invalid', json.dumps(frame())],0)
        self.assertEqual(s['input']['malformed_lines'],1)
    def test_sampled_command_not_speed(self):
        f=frame();f['movement']['applied']=[.04,0,0]
        s=self.summary([f]);self.assertEqual(s['command_nonzero_frames'],1)
        self.assertNotIn('measured_speed',s)
    def test_bad_gravity_norm(self):
        f=frame();f['safety']['gravity']=[0,0,-9.81]
        self.assertIsNone(self.summary([f])['tilt_from_gravity_deg'])
    def test_summary_emits_standard_json(self):
        json.dumps(self.summary([frame()]),allow_nan=False)

if __name__=='__main__':unittest.main()
