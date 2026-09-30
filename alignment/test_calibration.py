import unittest

import cv2
import numpy as np

from calibrate_extrinsics import average_transforms, detect_corners, pose_to_matrix, relative_transform


class CalibrationTests(unittest.TestCase):
    def test_board_detectable_at_tof_resolution(self):
        board = np.full((180, 240), 255, dtype=np.uint8)
        squares = (np.indices((7, 10)).sum(axis=0) % 2 * 255).astype(np.uint8)
        board[34:146, 40:200] = np.repeat(np.repeat(squares, 16, axis=0), 16, axis=1)

        corners = detect_corners(board, (9, 6))

        self.assertIsNotNone(corners)
        self.assertEqual(corners.shape, (54, 1, 2))
        points = np.zeros((54, 3), dtype=np.float32)
        points[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 25
        intrinsic = np.array([[190., 0, 120.], [0, 190., 90.], [0, 0, 1.]])
        ok, _, translation = cv2.solvePnP(points, corners, intrinsic, np.zeros(5))
        self.assertTrue(ok)
        self.assertTrue(np.all(np.isfinite(translation)))

    def test_relative_transform_maps_tof_points_into_rgb(self):
        rgb_board = pose_to_matrix(np.zeros((3, 1)), np.array([[40.], [0.], [500.]]))
        tof_board = pose_to_matrix(np.zeros((3, 1)), np.array([[10.], [0.], [500.]]))

        transform = relative_transform(rgb_board, tof_board)

        np.testing.assert_allclose(transform[:3, 3], [30, 0, 0])
        np.testing.assert_allclose(transform @ tof_board, rgb_board)

    def test_averaging_preserves_rigid_transform(self):
        transform = pose_to_matrix(np.array([[0.], [0.], [0.1]]), np.array([[30.], [2.], [1.]]))

        result = average_transforms([transform, transform])

        np.testing.assert_allclose(result, transform, atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(result[:3, :3]), 1.0)


if __name__ == "__main__":
    unittest.main()