"""
teleop/utils/weighted_moving_filter.py
Weighted moving average filter for smoothing XR tracking data and retargeted joint commands.
Reused and adapted from unitreerobotics/xr_teleoperate.
"""

import numpy as np


class WeightedMovingFilter:
    """
    Exponential / weighted moving window filter for smoothing multi-dimensional signals.
    """

    def __init__(self, weights=(0.5, 0.3, 0.2), data_size: int = 6):
        self._weights = np.array(weights, dtype=np.float64)
        total_w = np.sum(self._weights)
        if not np.isclose(total_w, 1.0):
            self._weights = self._weights / total_w
        self._window_size = len(self._weights)
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size, dtype=np.float64)
        self._data_queue = []

    def reset(self):
        self._data_queue.clear()
        self._filtered_data = np.zeros(self._data_size, dtype=np.float64)

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return np.array(self._data_queue[-1], dtype=np.float64)
        data_array = np.array(self._data_queue, dtype=np.float64)
        return np.sum(data_array * self._weights[::-1, None], axis=0)

    def add_data(self, new_data):
        new_data = np.asarray(new_data, dtype=np.float64)
        assert len(new_data) == self._data_size, f"Data size mismatch: got {len(new_data)}, expected {self._data_size}"

        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data.copy())
        self._filtered_data = self._apply_filter()
        return self._filtered_data

    @property
    def filtered_data(self):
        return self._filtered_data
