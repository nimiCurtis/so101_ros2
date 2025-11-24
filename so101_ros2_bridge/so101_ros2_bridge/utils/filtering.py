from __future__ import annotations


class LowPassFilter:
    """
    A simple low-pass filter implementation.
    """

    def __init__(self, alpha):
        """Initialize the low-pass filter with a smoothing factor alpha (0 < alpha < 1).

        Args:
            alpha (float): Smoothing factor between 0 and 1. Lower values result in more smoothing.

        """
        if alpha < 0.0 or alpha > 1.0:
            raise ValueError('[LowPassFilter]: Smoothing factor must be in the range (0, 1)')
        self.alpha = alpha
        self.y_prev = None

    def reset(self):
        self.y_prev = None

    def filter(self, x):
        if self.y_prev is None:
            self.y_prev = x
        y = self.y_prev + self.alpha * (x - self.y_prev)
        self.y_prev = y
        return y
