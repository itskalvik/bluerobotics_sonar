from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Optional

import numpy as np
from collections import deque
from scipy.signal import find_peaks


class Ping1DFilter:
    def __init__(self, threshold):
        self.threshold = threshold

    def __call__(self, distance, confidence):
        if confidence < self.threshold:
            return distance
        else:
            return 0.2
        

class SonarStabilityFilter:
    """
    Hold-last-value filter for 1D sonar readings.

    The filter appends each new sample into a fixed-size window, computes the
    sum of the absolute second difference (MASD) as a jitter metric, and:
      - if the metric is ABOVE `threshold`, returns the previous stable value;
      - otherwise updates and returns the stable value.

    This is useful for suppressing brief spikes/ringing while letting smooth,
    consistent changes through.

    Parameters
    ----------
    window_size : int, default=15
        Number of recent samples used to assess stability (>= 3 recommended).
    threshold : float, default=0.2
        MASD threshold. Lower = stricter (more holding), higher = looser.
        Using a mean (not sum) makes this largely independent of window_size.
    """

    def __init__(
        self,
        window_size: int = 15,
        threshold: float = 0.2,
    ) -> None:
        if not isinstance(window_size, int) or window_size < 3:
            raise ValueError("window_size must be an integer >= 3.")
        if not np.isfinite(threshold) or threshold < 0:
            raise ValueError("threshold must be a non-negative finite number.")

        self.window_size = window_size
        self.threshold = float(threshold)
        self._buf: deque = deque(np.zeros(3), maxlen=window_size)
        self._stable: Optional[float] = 0.5

    def _metric(self) -> float:
        """
        Sum of the absolute second difference (MASD) over the current buffer.
        """
        arr = np.asarray(self._buf)
        diff2 = np.diff(arr, n=2)
        return np.sum(np.abs(diff2))

    def __call__(self, x: float) -> float:
        """
        Ingest one sample and return the filtered (held/updated) value.
        """
        if x < 0:
            return self._stable

        # Fill buffer
        self._buf.append(x)

        # Compute stability metric and decide to hold or update
        metric = self._metric()
        if metric > self.threshold:
            # Too jittery → hold
            return self._stable
        else:
            # Stable enough → update
            self._stable = x
            return self._stable


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
    scan_threshold : float, default=100.0
        Minimum peak height (in the convolved difference signal) to count as a detection.
    window_size : int, default=5
        Size of the end-difference window used to emphasize rising edges. Must be >= 2.
    """

    max_range: float = 1.0
    offset: int = 20
    scan_threshold: float = 100.0
    window_size: int = 5

    def __post_init__(self) -> None:
        # Basic parameter validation
        if not np.isfinite(self.max_range) or self.max_range <= 0:
            raise ValueError("max_range must be a positive finite number.")
        if not isinstance(self.offset, int) or self.offset < 0:
            raise ValueError("offset must be a non-negative integer.")
        if not np.isfinite(self.scan_threshold) or self.scan_threshold < 0:
            raise ValueError("scan_threshold must be a non-negative finite number.")
        if not isinstance(self.window_size, int) or self.window_size < 2:
            raise ValueError("window_size must be an integer >= 2.")

        # End-difference kernel:  [1, 0, 0, ..., 0, -1]
        # More robust than first-difference for small-window noise.
        window = np.zeros(self.window_size, dtype=float)
        window[0], window[-1] = 1.0, -1.0
        self._window = window

    def __call__(self, data: Sequence[int], dist_ref: float = 0.0) -> float:
        """
        Estimate distance to the nearest object from a 1D sonar intensity scan.

        Parameters
        ----------
        data : Sequence[int]
            Raw sonar intensity values (index increases with range).
        dist_ref : float, default=0.0
            Optional prior/reference distance (same units as output).
            If > 0, the detected peak closest to this distance is returned.

        Returns
        -------
        float
            Estimated distance in the same units as `max_range`.
            Returns -1.0 if no valid detection is found or input is invalid.
        """
        # ---- Input coercion & sanity checks -----------------------------------
        try:
            arr = np.asarray(data, dtype=np.uint8).ravel()
        except Exception:
            return -1.0
        n = arr.size

        # ---- Preprocess & edge-contrast filter -------------------------------
        # Ignore near-field bins
        arr = arr[self.offset:]

        # Convolve to emphasize rising edges; 'valid' avoids padding artifacts
        diff = np.convolve(arr, self._window, mode="valid")
        # diff length is: n - offset - (window_size - 1)
        m = diff.size

        # ---- Peak picking ----------------------------------------------------
        # Height threshold in the filtered domain
        peaks, props = find_peaks(diff, height=self.scan_threshold)

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
