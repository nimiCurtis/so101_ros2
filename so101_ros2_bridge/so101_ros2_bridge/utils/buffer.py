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

from typing import List, Optional, Sequence

import numpy as np

from .filtering import LowPassFilter


class ActionBuffer:
    """Buffer for sequentially consuming action chunks.

    This class stores a list of actions (e.g. joint position vectors),
    keeps an index into that list, and returns actions one-by-one via
    :meth:`get`.

    Optionally, a low-pass filter can be applied to each returned action.
    """

    def __init__(self, lpf: Optional[LowPassFilter] = None) -> None:
        """Initialize an empty action buffer.

        Args:
            lpf: Optional low-pass filter applied on each returned action.
        """
        self._actions: List[List[float]] = []
        self._index: int = 0
        self._filter = lpf

    @property
    def index(self) -> int:
        """Current index into the buffer."""
        return self._index

    @property
    def is_empty(self) -> bool:
        """Whether the buffer currently holds no actions."""
        return len(self._actions) == 0

    def set(
        self,
        actions: Sequence[Sequence[float]],
        start_index: int = 0,
    ) -> None:
        """Set a new action chunk and starting index.

        Args:
            actions: 2D sequence of actions, shape [T, n_joints].
            start_index: Index to start from when :meth:`get` is called.
                Values are clamped to [0, len(actions) - 1] when non-empty.
        """
        # Convert to plain Python lists
        self._actions = [list(a) for a in actions]

        if not self._actions:
            # Empty buffer: just reset index
            self._index = 0
            return

        # Clamp start_index to valid range
        if start_index < 0:
            self._index = 0
        elif start_index >= len(self._actions):
            self._index = len(self._actions) - 1
        else:
            self._index = start_index

    def get(self) -> Optional[List[float]]:
        """Return the current action and advance the index.

        The behavior matches the previous `get_action` logic:
        - If the buffer is empty, returns None.
        - If the index is past the end, it is clamped to the last element.
        - The index advances until the last element, then stays there.

        Returns:
            The next action as a list of floats, or None if buffer is empty.
        """
        if not self._actions:
            return None

        # Clamp index if needed
        if self._index >= len(self._actions):
            self._index = len(self._actions) - 1

        # Get current action
        current = self._actions[self._index]

        # Advance index while not at the last element
        if self._index < len(self._actions) - 1:
            self._index += 1

        # Apply optional low-pass filter
        if self._filter is None:
            return current

        return self._filter.filter(np.array(current)).tolist()

    def reset(self) -> None:
        """Clear all actions and reset the index."""
        self._actions = []
        self._index = 0
        if self._filter is not None:
            self._filter.reset()
