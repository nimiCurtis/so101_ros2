# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations

from typing import Any


class LowPassFilter:
    """
    A simple low-pass filter implementation.
    """

    def __init__(self, alpha: float):
        """Initialize the low-pass filter with a smoothing factor alpha (0 < alpha < 1).

        Args:
            alpha (float): Smoothing factor between 0 and 1. Lower values result in more smoothing.

        """
        if alpha < 0.0 or alpha > 1.0:
            raise ValueError('[LowPassFilter]: Smoothing factor must be in the range (0, 1)')
        self.alpha = alpha
        self.y_prev = None

    def reset(self) -> None:
        """Reset the filter state."""
        self.y_prev = None

    def filter(self, x: Any) -> Any:
        """Apply the low-pass filter to the input value x.
        Args:
            x (Any): The new input value to filter.
        Returns:
            Any: The filtered output value.
        """
        if self.y_prev is None:
            self.y_prev = x
        y = self.y_prev + self.alpha * (x - self.y_prev)
        self.y_prev = y
        return y
