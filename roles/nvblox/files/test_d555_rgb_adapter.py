"""Pure conversion/geometry tests for d555_rgb_adapter (no ROS required)."""

import unittest

import numpy as np

from d555_rgb_adapter import (
    Rectifier,
    canonical_frame,
    parse_args,
    pinhole_from_k,
    rectified_camera_info_fields,
    rgb8_view,
    yuy2_to_rgb,
)

# The D555's colour intrinsics observed on the bench (896x504).
K_COLOR = [450.17462158203125, 0.0, 439.2585754394531,
           0.0, 449.70281982421875, 246.913818359375,
           0.0, 0.0, 1.0]
D_COLOR = [-0.05273808538913727, 0.05760319158434868,
           -0.00013026213855482638, 0.0005918419337831438, -0.017927037551999092]
WIDTH, HEIGHT = 896, 504


class FrameTests(unittest.TestCase):
    def test_expected_name_with_optional_nul_padding(self):
        for padding in ("", "\x00", "\x00" * 37):
            self.assertEqual(canonical_frame("camera_color_optical_frame" + padding),
                             "camera_color_optical_frame")

    def test_other_names_are_rejected(self):
        for bad in ("camera_depth_optical_frame", "", " camera_color_optical_frame",
                    "camera_color_optical_frame ", None, "camera_color_optical_frame\x00x"):
            with self.assertRaises(ValueError):
                canonical_frame(bad)


class CameraInfoTests(unittest.TestCase):
    def test_pinhole_extraction(self):
        self.assertEqual(pinhole_from_k(K_COLOR), (K_COLOR[0], K_COLOR[4], K_COLOR[2], K_COLOR[5]))

    def test_non_pinhole_matrices_are_rejected(self):
        for bad in (K_COLOR[:8], [0.0] + K_COLOR[1:], [float("nan")] + K_COLOR[1:],
                    K_COLOR[:1] + [1.0] + K_COLOR[2:], K_COLOR[:8] + [2.0]):
            with self.assertRaises(ValueError):
                pinhole_from_k(bad)

    def test_rectified_fields_have_zero_distortion_and_matching_projection(self):
        fields = rectified_camera_info_fields(K_COLOR, WIDTH, HEIGHT)
        self.assertEqual(fields["width"], WIDTH)
        self.assertEqual(fields["height"], HEIGHT)
        self.assertEqual(fields["distortion_model"], "plumb_bob")
        self.assertEqual(fields["d"], [0.0] * 5)
        self.assertEqual(fields["k"], K_COLOR)
        self.assertEqual(fields["r"], [1, 0, 0, 0, 1, 0, 0, 0, 1])
        fx, fy, cx, cy = K_COLOR[0], K_COLOR[4], K_COLOR[2], K_COLOR[5]
        self.assertEqual(fields["p"], [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0])
        with self.assertRaises(ValueError):
            rectified_camera_info_fields(K_COLOR, 0, HEIGHT)


class Yuy2Tests(unittest.TestCase):
    def test_gray_frame_converts_to_gray_rgb(self):
        width, height = 8, 4
        payload = bytes([128, 128] * (width // 2) * 2) * height  # Y=U=V=128 everywhere
        self.assertEqual(len(payload), 2 * width * height)
        rgb = yuy2_to_rgb(payload, width, height, 2 * width)
        self.assertEqual(rgb.shape, (height, width, 3))
        self.assertEqual(rgb.dtype, np.uint8)
        spread = rgb.max(axis=2).astype(int) - rgb.min(axis=2).astype(int)
        self.assertLessEqual(int(spread.max()), 8, "neutral chroma must give gray")
        self.assertTrue(112 <= int(rgb.mean()) <= 144)

    def test_luma_order_is_preserved(self):
        width, height = 4, 1
        # Two pixel pairs: dark, then bright (Y0 U Y1 V).
        payload = bytes([16, 128, 16, 128, 235, 128, 235, 128])
        rgb = yuy2_to_rgb(payload, width, height, 2 * width)
        self.assertLess(int(rgb[0, 0].mean()), int(rgb[0, 3].mean()))

    def test_step_and_payload_size_are_checked(self):
        with self.assertRaises(ValueError):
            yuy2_to_rgb(bytes(16), 4, 2, 12)
        with self.assertRaises(ValueError):
            yuy2_to_rgb(bytes(15), 4, 2, 8)
        with self.assertRaises(ValueError):
            rgb8_view(bytes(23), 4, 2, 12)


class RectifierTests(unittest.TestCase):
    def test_zero_distortion_is_identity(self):
        rectifier = Rectifier(K_COLOR, [0.0] * 5, WIDTH, HEIGHT)
        image = np.random.default_rng(0).integers(0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.assertIs(rectifier.apply(image), image)

    def test_bench_distortion_moves_periphery_and_keeps_principal_point(self):
        rectifier = Rectifier(K_COLOR, D_COLOR, WIDTH, HEIGHT)
        self.assertFalse(rectifier.identity)
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        cx, cy = int(round(K_COLOR[2])), int(round(K_COLOR[5]))
        image[cy - 3:cy + 4, cx - 3:cx + 4] = 255      # a block at the principal point
        image[8:16, 8:16] = 255                        # a block near a corner
        out = rectifier.apply(image)
        self.assertEqual(out.shape, image.shape)
        self.assertGreater(int(out[cy, cx].mean()), 200, "principal point must stay put")
        self.assertFalse(np.array_equal(out[0:32, 0:32], image[0:32, 0:32]),
                         "the corner must be remapped by the radial model")

    def test_size_mismatch_and_bad_distortion_are_rejected(self):
        rectifier = Rectifier(K_COLOR, D_COLOR, WIDTH, HEIGHT)
        with self.assertRaises(ValueError):
            rectifier.apply(np.zeros((HEIGHT // 2, WIDTH, 3), dtype=np.uint8))
        with self.assertRaises(ValueError):
            Rectifier(K_COLOR, [0.1, 0.2, 0.3], WIDTH, HEIGHT)
        with self.assertRaises(ValueError):
            Rectifier(K_COLOR, [float("inf")] * 5, WIDTH, HEIGHT)


class ArgumentTests(unittest.TestCase):
    def test_defaults_follow_the_serial(self):
        options, ros_args = parse_args(["--serial", "123", "--ros-args", "-p", "x:=1"])
        self.assertEqual(options.image_topic, "/realsense/D555_123_Color")
        self.assertEqual(options.camera_info_topic, "/d555/color/camera_info")
        self.assertEqual(options.output_namespace, "/d555/color/rect")
        self.assertEqual(options.max_rate, 5.0)
        self.assertEqual(ros_args, ["--ros-args", "-p", "x:=1"])

    def test_bad_serial_and_rate_are_rejected(self):
        with self.assertRaises(SystemExit):
            parse_args(["--serial", "12a"])
        with self.assertRaises(SystemExit):
            parse_args(["--max-rate", "-1"])


if __name__ == "__main__":
    unittest.main()
