import math
import unittest

from lidar_py.slam_correction_logic import Pose2D, SlamCorrectionDetector


def detector():
    return SlamCorrectionDetector(
        translation_threshold=0.10,
        yaw_threshold=math.radians(0.5),
        window_sec=0.5,
        window_translation_threshold=0.15,
        window_yaw_threshold=math.radians(1.0),
        max_sample_gap_sec=1.0,
    )


class SlamCorrectionDetectorTest(unittest.TestCase):
    def test_small_updates_do_not_trigger(self):
        subject = detector()
        self.assertIsNone(subject.update(Pose2D(0.0, 0.0, 0.0, 0.0)))
        self.assertIsNone(subject.update(Pose2D(0.1, 0.02, 0.0, 0.002)))
        self.assertIsNone(subject.update(Pose2D(0.2, 0.04, 0.0, 0.004)))

    def test_instant_translation_triggers(self):
        subject = detector()
        subject.update(Pose2D(0.0, 0.0, 0.0, 0.0))
        correction = subject.update(Pose2D(0.1, 0.12, 0.0, 0.0))
        self.assertIsNotNone(correction)
        self.assertAlmostEqual(correction.instant_translation, 0.12)

    def test_instant_yaw_triggers(self):
        subject = detector()
        subject.update(Pose2D(0.0, 0.0, 0.0, 0.0))
        correction = subject.update(
            Pose2D(0.1, 0.0, 0.0, math.radians(0.6)))
        self.assertIsNotNone(correction)

    def test_window_translation_triggers(self):
        subject = detector()
        subject.update(Pose2D(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(subject.update(Pose2D(0.1, 0.06, 0.0, 0.0)))
        self.assertIsNone(subject.update(Pose2D(0.2, 0.12, 0.0, 0.0)))
        correction = subject.update(Pose2D(0.3, 0.18, 0.0, 0.0))
        self.assertIsNotNone(correction)
        self.assertAlmostEqual(correction.window_translation, 0.18)

    def test_window_yaw_triggers_for_small_incremental_corrections(self):
        subject = detector()
        subject.update(Pose2D(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(subject.update(
            Pose2D(0.1, 0.0, 0.0, math.radians(0.4))))
        self.assertIsNone(subject.update(
            Pose2D(0.2, 0.0, 0.0, math.radians(0.8))))
        correction = subject.update(
            Pose2D(0.3, 0.0, 0.0, math.radians(1.2)))
        self.assertIsNotNone(correction)
        self.assertAlmostEqual(
            math.degrees(correction.window_yaw), 1.2)

    def test_angle_wrap_and_long_gap_reset(self):
        subject = detector()
        subject.update(Pose2D(0.0, 0.0, 0.0, math.radians(179.9)))
        self.assertIsNone(
            subject.update(Pose2D(0.1, 0.0, 0.0, math.radians(-179.9))))
        self.assertIsNone(subject.update(Pose2D(2.0, 1.0, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
