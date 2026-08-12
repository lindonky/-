import tempfile
import unittest
from pathlib import Path

import analyze_calibration_capture as analyzer


HEADER = (
    "timestamp_ms,elapsed_ms,sequence,roll_deg,pitch_deg,yaw_deg,motion_deg,lateral_deg,"
    "gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,pitch_out,roll_out,"
    "t1_pulse_us,t2_pulse_us,t3_pulse_us,mode,imu_valid,estop,stabilize\n"
)


def row(t, elapsed, seq, pitch, motion, lateral, t2=1500, mode=1):
    return (
        f"{t},{elapsed},{seq},{lateral:.2f},{pitch:.2f},0,{motion:.2f},{lateral:.2f},"
        f"1,{motion * 2:.1f},0,0,0,1,0,10,1490,{t2},1510,{mode},1,0,1\n"
    )


def capture(rows, label="normal-lift"):
    return (
        "\ufeff# schema=alwaysbea-calibration-capture-v1\n"
        f"# label={label}\n# device_boot=123\n" + HEADER + "".join(rows)
    )


class AnalyzeCaptureTest(unittest.TestCase):
    def load(self, text):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "capture.csv"
            path.write_text(text, encoding="utf-8")
            return analyzer.load_capture(path)

    def test_clean_active_capture(self):
        metadata, rows = self.load(capture([
            row(1000, 0, 10, 0, 0, 0),
            row(1050, 50, 11, 8, 8, 1),
            row(1100, 100, 12, 20, 20, 2),
            row(1150, 150, 13, 0, 0, 0),
        ]))
        report = analyzer.analyze(metadata, rows)
        self.assertAlmostEqual(report["measured_rate_hz"], 20.0)
        self.assertEqual(report["timing"]["missing_sequence_values"], 0)
        self.assertEqual(report["active_mode"]["t2_violations"], 0)
        self.assertEqual(report["pwm_us"]["t1_pulse_us"]["min"], 1490)
        self.assertEqual(report["pwm_us"]["t2_pulse_us"]["mean"], 1500)
        self.assertEqual(report["pwm_us"]["t3_pulse_us"]["max"], 1510)
        self.assertEqual(report["warnings"], [])

    def test_quality_and_safety_warnings(self):
        metadata, rows = self.load(capture([
            row(1000, 0, 20, 0, 0, 0),
            row(1200, 200, 22, 1, 1, 5, t2=1600),
            row(1200, 200, 22, 1, 1, 6, t2=1600),
        ]))
        report = analyzer.analyze(metadata, rows)
        self.assertEqual(report["timing"]["missing_sequence_values"], 1)
        self.assertEqual(report["timing"]["nonpositive_dt"], 1)
        self.assertEqual(report["active_mode"]["t2_violations"], 2)
        self.assertGreaterEqual(len(report["warnings"]), 4)

    def test_missing_schema_and_columns(self):
        with self.assertRaises(analyzer.CaptureError):
            self.load("a,b\n1,2\n")
        with self.assertRaises(analyzer.CaptureError):
            self.load("# schema=alwaysbea-calibration-capture-v1\na,b\n1,2\n")

    def test_csv_export_metadata_and_client_prefix(self):
        metadata, rows = self.load(
            "\ufeff# label=fast-lift\n# exported_at=2026-08-12T00:00:00Z\n"
            "# schema=alwaysbea-calibration-capture-v1\n"
            "# device_boot=456,samples=2,duration_ms=50,rate_hz=20.0,full=0\n"
            + HEADER
            + row(1000, 0, 1, 0, 0, 0)
            + row(1050, 50, 2, 8, 8, 1)
        )
        self.assertEqual(metadata["label"], "fast-lift")
        self.assertEqual(metadata["device_boot"], "456")
        self.assertEqual(metadata["samples"], "2")
        self.assertEqual(len(rows), 2)


if __name__ == "__main__":
    unittest.main()
