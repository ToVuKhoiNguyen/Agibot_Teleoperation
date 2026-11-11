"""Tracks target movement and detects arrival when |sent - target| < 0.01."""
import numpy as np

class ArrivalDetector:
    def __init__(self, tol: float = 0.01, min_move: float = 0.03):
        self.tol = tol
        self.min_move = min_move
        self.has_moved = False
        self.arrived = False
        self.init_target = None

    def update(self, sent_q: np.ndarray, target_q: np.ndarray, seq: int) -> tuple[bool, bool]:
        """Returns (is_arrived_flag, is_first_arrival_event)."""
        if self.init_target is None:
            self.init_target = target_q.copy()
        if not self.has_moved:
            if float(np.max(np.abs(target_q - self.init_target))) > self.min_move:
                self.has_moved = True
                self.arrived = False

        err = float(np.max(np.abs(sent_q - target_q)))
        is_arrived = (err < self.tol)
        is_first_event = False
        if self.has_moved and is_arrived and not self.arrived:
            self.arrived = True
            is_first_event = True
        return is_arrived, is_first_event
