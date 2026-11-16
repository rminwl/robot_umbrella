"""Run: python -m unittest discover -s Software/preprocessing -v"""
import contextlib
import io
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pandas as pd

import wind_logger as wl


class LoggerTests(unittest.TestCase):
    def test_existing_selftest(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(wl.run_selftest())

    def test_all_joint_limits(self):
        for mode in wl.NEUTRAL_POSE:
            for direction in range(360):
                for tilt in (-10, 0, 15, 30, 100):
                    targets = wl.map_rib_targets(direction, tilt, mode=mode)
                    for key, value in targets.items():
                        low, high = (wl.J1_MIN, wl.J1_MAX) if key.endswith('1') else (wl.J2_MIN, wl.J2_MAX)
                        self.assertTrue(low <= value <= high)

    def test_nonfinite_prediction_neutral_and_safe_priority(self):
        for direction, tilt in ((math.nan, 10), (0, math.inf)):
            self.assertTrue(all(v == 0 for v in wl.map_rib_targets(direction, tilt).values()))
            safe = wl.map_rib_targets(direction, tilt, safe_mode=True)
            self.assertEqual(safe['r1_j1'], wl.J1_SAFE)

    def test_fault_pose_and_legacy_csv(self):
        rows = pd.DataFrame([dict(target_tilt_dir_deg=90, target_tilt_deg=20,
                                  direction_valid=0, safe_mode=0, pose_safe=1)])
        self.assertEqual(wl.add_joint_columns(rows).iloc[0]['pred_r1_j1'], wl.J1_SAFE)
        self.assertEqual(wl.add_joint_columns(rows.drop(columns='pose_safe')).iloc[0]['pred_r1_j1'], 0)

    def test_nan_direction_flag_is_not_true(self):
        rows = pd.DataFrame([dict(target_tilt_dir_deg=90, target_tilt_deg=20,
                                  direction_valid=np.nan)])
        self.assertEqual(wl.add_joint_columns(rows).iloc[0]['pred_r1_j1'], 0)

    def test_features_filter_nonfinite_and_align_labels(self):
        rows = pd.DataFrame([[0, 1, 2, 3, .9]] * 4, columns=wl.FEATURE_COLUMNS, dtype=float)
        rows['valid'] = 1
        rows['direction_valid'] = 1
        rows[wl.LABEL_COLUMNS[0]] = [90, 180, np.inf, np.nan]
        rows[wl.LABEL_COLUMNS[1]] = 20
        rows.loc[1, 'gust_ms'] = np.inf
        x, y = wl.build_features(rows)
        self.assertEqual(x.shape, (1, 5))
        np.testing.assert_array_equal(y, [[90, 20]])

    def test_error_report_uses_observed_deltas_and_splits_reset(self):
        rows = pd.DataFrame(dict(t_ms=[10000, 10200, 0, 200], valid=[0]*4,
                                 vane_status=['OPEN']*4, err_events=[10, 10, 1, 1],
                                 err_total_ms=[5000, 5200, 100, 300], err_max_ms=[5000]*4))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            segments = wl.error_report(rows)
        self.assertIn('누적 0.4 s', output.getvalue())
        self.assertIn('100.0%', output.getvalue())
        self.assertEqual(len(segments), 2)

    def test_serial_schema_change_closes_port(self):
        class FakeSerial:
            closed = False
            def reset_input_buffer(self): pass
            def write(self, data): pass
            def readline(self):
                return next(self.lines)
            def close(self): self.closed = True
        device = FakeSerial()
        device.lines = iter([b't_ms,valid\n', b'bad,1\n', b'NaN,1\n', b'0,1\n', b't_ms,other\n'])
        with tempfile.TemporaryDirectory() as folder:
            with patch('serial.Serial', return_value=device), patch.object(wl.time, 'sleep'):
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                    wl.log_serial('FAKE', out_dir=folder)
            self.assertTrue(device.closed)
            content = next(Path(folder).glob('*.csv')).read_text()
            self.assertIn('0,1,,', content)
            self.assertNotIn('bad', content)
            self.assertNotIn('NaN', content)

    def test_calibration_rows_are_independent(self):
        self.assertIsNot(wl.JOINT_ZERO_TICK[0], wl.JOINT_ZERO_TICK[1])
        self.assertIsNot(wl.JOINT_DIR_SIGN[0], wl.JOINT_DIR_SIGN[1])


if __name__ == '__main__':
    unittest.main()
