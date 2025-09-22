from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Optional

import numpy as np
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
from filterpy.kalman import KalmanFilter


class KF():
    def __init__(self,
                 dt=0.1,
                 noise=1.0):
        self.dt = dt
        self.noise = noise
        
        # KF to track the center (position, velocity, acceleration) 
        # from position and acceleration data
        self.init_kf()

    def __call__(self, x):
        self.kf.update(np.array([x]).reshape(1, 1))
        self.kf.predict()
        sol = self.kf.x.reshape(-1)[0]
        return sol

    def init_kf(self):
        # dim_x state dim
        # dim_z measurement data dim
        self.kf = KalmanFilter(dim_x=1, dim_z=1)
        dt = self.dt

        # Initial state
        self.kf.x = np.eye(1)*0.
        
        # State transition matrix (dim_x, dim_x)
        self.kf.F = np.eye(1)

        # Measurement function (dim_z, dim_x)
        # Only observe the position and acceleration 
        self.kf.H = np.eye(1)

        # State covariance matrix (dim_x x dim_x)
        self.kf.P *= np.eye(1)

        # Measurement noise (dim_z x dim_z)
        self.kf.R = np.eye(1)*self.noise


@dataclass
class SonarRangeFinder:
    """
    Estimate distance to the nearest object from a 1D sonar intensity scan.

    Parameters
    ----------
    max_range : float, default=1.0
        Maximum measurable range corresponding to the end of `data`
        (same units as the returned distance).
    offset : int, default=20
        Number of initial bins to ignore (e.g., transducer ring-down / near-field saturation).
    threshold : float, default=100.0
        Minimum peak height (in the convolved difference signal) to count as a detection.
    window_size : int, default=5
        Size of the end-difference window used to emphasize rising edges. Must be >= 2.
    """

    max_range: float = 1.0
    offset: int = 25
    threshold: float = 50.0
    window_size: int = 15
    sigma: float = 3.0

    def __post_init__(self) -> None:
        # Basic parameter validation
        if not np.isfinite(self.max_range) or self.max_range <= 0:
            raise ValueError("max_range must be a positive finite number.")
        if not isinstance(self.offset, int) or self.offset < 0:
            raise ValueError("offset must be a non-negative integer.")
        if not np.isfinite(self.threshold) or self.threshold < 0:
            raise ValueError("threshold must be a non-negative finite number.")
        if not isinstance(self.window_size, int) or self.window_size < 2:
            raise ValueError("window_size must be an integer >= 2.")
        if not  np.isfinite(self.sigma) or self.sigma < 0.:
            raise ValueError("sigma must be a positive finite number.")

        # End-difference kernel:  [1, 0, 0, ..., 0, -1]
        # More robust than first-difference for small-window noise.
        self._window = np.linspace(1, -1, self.window_size)

    def __call__(self, data: Sequence[int], dist_ref: float = 0.0) -> float:
        # ---- Input coercion & sanity checks -----------------------------------
        try:
            arr = np.asarray(data, dtype=np.uint8).ravel()
        except Exception:
            return -1.0
        n = arr.size

        # ---- Preprocess & edge-contrast filter -------------------------------
        # Ignore near-field bins
        arr = arr[self.offset:]

        gauss = gaussian_filter1d(arr, sigma=self.sigma, mode='nearest')

        # Convolve to emphasize rising edges; 'valid' avoids padding artifacts
        diff = np.convolve(gauss, self._window, mode="valid")
        # diff length is: n - offset - (window_size - 1)
        m = diff.size
        diff[diff < 0] = 0

        # ---- Peak picking ----------------------------------------------------
        # Height threshold in the filtered domain
        peaks, properties = find_peaks(diff, 
                                       height=self.threshold,
                                       width=0.1)

        if peaks.size == 0:
            return -1.0

        # ---- Peak selection strategy -----------------------------------------
        # If a reference distance is provided, choose the peak closest to it.
        if dist_ref and dist_ref > 0.0 and np.isfinite(dist_ref):
            # Map reference distance (in meters/whatever) to an index in the *diff* domain
            # Absolute-bin estimate in original array coordinates:
            # (we map 0..n-1 -> 0..max_range). Use (n-1) to avoid a slight end bias.
            ref_abs_bin = (dist_ref / self.max_range) * (n - 1)
            # Convert to diff-index (which aligns to offset + k + (window_size - 1))
            ref_diff_idx = ref_abs_bin - self.offset - (self.window_size - 1)
            # Choose the peak closest to ref, after clipping to valid range
            ref_diff_idx = np.clip(ref_diff_idx, 0, m - 1)
            sel = np.argmin(np.abs(peaks - ref_diff_idx))
            peak = peaks[sel]
        else:
            # Nearest object normally corresponds to the earliest valid peak
            peak = peaks[0]

        # ---- Map selected peak back to distance ------------------------------
        # Peak is in the diff domain; convert to an absolute bin in the original array.
        abs_bin = self.offset + peak + (self.window_size - 1)

        # Guard (shouldn't happen, but safe):
        abs_bin = np.clip(abs_bin, 0, n - 1)

        # Map bin to distance proportionally along the max range.
        # Use (n - 1) so that the last bin maps exactly to max_range.
        dist = (abs_bin / float(max(n - 1, 1))) * self.max_range

        return float(dist)