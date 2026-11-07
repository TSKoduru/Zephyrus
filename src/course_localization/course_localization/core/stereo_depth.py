"""Depth inferred from rectified stereo imagery, never from oracle geometry.

OpenCV StereoSGBM is a host-side approximation, not the OAK firmware matcher.
The output is aligned to the RIGHT optical camera, optical-axis Z in metres.
Blank/ambiguous/occluded pixels remain invalid; no hole filling is performed.
"""

import numpy as np
import cv2
from .depth_sensor import DepthFrame


class StereoDepthEstimator:
    def __init__(
        self,
        K,
        baseline_m=0.075,
        output_size=(160, 100),
        min_range=0.2,
        max_range=4.0,
        lr_tolerance_px=1.0,
        texture_std_min=2.0,
        min_valid_fraction=0.25,
        texture_noise_multiplier=0.0,
    ):
        self.K = np.asarray(K, dtype=float).copy()
        self.baseline_m = float(baseline_m)
        self.output_size = tuple(output_size)
        self.min_range, self.max_range = float(min_range), float(max_range)
        self.lr_tolerance_px = float(lr_tolerance_px)
        self.texture_std_min = float(texture_std_min)
        self.min_valid_fraction = float(min_valid_fraction)
        self.texture_noise_multiplier = float(texture_noise_multiplier)
        if not np.isfinite(self.texture_noise_multiplier) or self.texture_noise_multiplier < 0:
            raise ValueError("Texture noise multiplier must be finite and nonnegative")
        if self.K.shape != (3, 3) or baseline_m <= 0 or not 0 < min_range < max_range:
            raise ValueError("Invalid stereo geometry/range")
        self.num_disparities = int(np.ceil((self.K[0, 0] * baseline_m / min_range + 1) / 16)) * 16
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=5,
            P1=8 * 25,
            P2=32 * 25,
            disp12MaxDiff=1,
            preFilterCap=31,
            uniquenessRatio=12,
            speckleWindowSize=60,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.last_diagnostics = {}

    def compute(self, left, right, timestamp, T_body_camera):
        left = np.asarray(left)
        right = np.asarray(right)
        if (
            left.shape != right.shape
            or left.ndim != 2
            or left.dtype != np.uint8
            or right.dtype != np.uint8
        ):
            raise ValueError("Rectified left/right must be equal-shape uint8 mono images")
        h, w = right.shape
        if w <= self.num_disparities + 8:
            raise ValueError("Stereo image too narrow for configured disparity search")
        d_left = (
            self.matcher.compute(np.ascontiguousarray(left), np.ascontiguousarray(right)).astype(
                np.float32
            )
            / 16
        )
        # Flipping and swapping makes right->left disparities positive too.
        d_right = (
            np.fliplr(
                self.matcher.compute(
                    np.ascontiguousarray(np.fliplr(right)), np.ascontiguousarray(np.fliplr(left))
                )
            ).astype(np.float32)
            / 16
        )
        y = np.arange(h)[:, None]
        x = np.arange(w, dtype=np.float32)[None, :]
        left_x = np.rint(x + d_right).astype(np.int32)
        inside = (left_x >= 0) & (left_x < w)
        corresponding = d_left[y, np.clip(left_x, 0, w - 1)]
        consistent = (
            inside
            & (d_right > 0)
            & (corresponding > 0)
            & (np.abs(d_right - corresponding) <= self.lr_tolerance_px)
        )
        gray = right.astype(np.float32)
        mean = cv2.boxFilter(gray, -1, (9, 9))
        variance = np.maximum(cv2.boxFilter(gray * gray, -1, (9, 9)) - mean * mean, 0)
        noise_sigma = 0.0
        if self.texture_noise_multiplier:
            from .image_noise_estimation import image_noise_sigma

            noise_sigma = image_noise_sigma(right)
        texture_threshold = max(self.texture_std_min, self.texture_noise_multiplier * noise_sigma)
        textured = variance >= texture_threshold**2
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = self.K[0, 0] * self.baseline_m / d_right
        valid = (
            consistent
            & textured
            & np.isfinite(depth)
            & (depth >= self.min_range)
            & (depth <= self.max_range)
        )
        depth[~valid] = np.nan
        ow, oh = self.output_size
        if w % ow or h % oh:
            raise ValueError("Output size must evenly divide input; no interpolated depth")
        bx, by = w // ow, h // oh
        blocks = depth.reshape(oh, by, ow, bx).transpose(0, 2, 1, 3).reshape(oh, ow, by * bx)
        finite = np.isfinite(blocks)
        count = finite.sum(axis=2)
        # Avoid all-NaN median warnings; explicit validity still owns the output.
        ordered = np.where(finite, blocks, np.inf)
        ordered.sort(axis=2)
        index = np.maximum(0, (count - 1) // 2)
        reduced = np.take_along_axis(ordered, index[..., None], axis=2)[..., 0]
        reduced[count < max(1, int(np.ceil(by * bx * self.min_valid_fraction)))] = np.nan
        K = self.K.copy()
        K[0, :] /= bx
        K[1, :] /= by
        K[0, 2] = (self.K[0, 2] + 0.5) / bx - 0.5
        K[1, 2] = (self.K[1, 2] + 0.5) / by - 0.5
        self.last_diagnostics = {
            "source": "rectified_images_stereo_sgbm",
            "full_valid_fraction": float(valid.mean()),
            "output_valid_fraction": float(np.isfinite(reduced).mean()),
            "lr_consistent_fraction": float(consistent.mean()),
            "textured_fraction": float(textured.mean()),
            "num_disparities": self.num_disparities,
            "image_noise_sigma": noise_sigma,
            "texture_std_threshold": texture_threshold,
        }
        return DepthFrame(
            float(timestamp), reduced.astype(np.float32), K, np.asarray(T_body_camera).copy()
        )
