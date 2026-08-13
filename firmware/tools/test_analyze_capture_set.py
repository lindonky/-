import unittest

import analyze_capture_set as analyzer


def sample(timestamp_ms, tilt_deg=0.0, accel_mag_g=1.0):
    return {
        "timestamp_ms": float(timestamp_ms),
        "tilt_deg": float(tilt_deg),
        "accel_mag_g": float(accel_mag_g),
    }


class TwoMethodFallReplayTest(unittest.TestCase):
    def test_normal_motion_does_not_trigger(self):
        report = analyzer._replay_two_method([
            sample(0, 0), sample(100, 18), sample(200, 26), sample(300, 0)
        ])
        self.assertFalse(report["confirmed"])
        self.assertEqual(report["candidate_reason"], "none")

    def test_short_angle_excursion_resets(self):
        report = analyzer._replay_two_method([
            sample(0, 68), sample(100, 68), sample(200, 68),
            sample(300, 0), sample(400, 68), sample(500, 68),
        ])
        self.assertFalse(report["confirmed"])

    def test_sustained_angle_triggers(self):
        report = analyzer._replay_two_method([
            sample(0, 68), sample(100, 68), sample(200, 68),
            sample(300, 68), sample(400, 68),
        ])
        self.assertTrue(report["confirmed"])
        self.assertEqual(report["candidate_reason"], "angle")
        self.assertEqual(report["confirmed_elapsed_ms"], 400)

    def test_low_or_high_acceleration_triggers_immediately(self):
        for accel in (0.55, 1.60):
            with self.subTest(accel=accel):
                report = analyzer._replay_two_method([sample(1000, 0, accel)])
                self.assertTrue(report["confirmed"])
                self.assertEqual(report["candidate_reason"], "acceleration")
                self.assertEqual(report["confirmed_elapsed_ms"], 0)


if __name__ == "__main__":
    unittest.main()
