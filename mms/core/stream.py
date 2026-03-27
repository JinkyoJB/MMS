# mms/core/stream.py

from __future__ import annotations

from collections import deque
from typing import Iterator, Optional

from mms.core.frames import Frame


class Stream:
    """
    Sliding-window buffer of Frame objects, ordered by insertion (timestamp).

    Keeps the most recent `max_size` frames. When the buffer is full,
    the oldest frame is automatically evicted on each new append.

    Example
    -------
    >>> stream = Stream(max_size=50)
    >>> stream.append(frame)
    >>> recent = stream.latest(5)
    >>> window = stream.since(time.time() - 2.0)
    """

    def __init__(self, max_size: int = 100) -> None:
        """
        Parameters
        ----------
        max_size : int, default=100
            Maximum number of frames retained. Oldest frames are evicted
            automatically when the buffer is full.
        """
        self._buffer: deque[Frame] = deque(maxlen=max_size)

    # ----- properties -----

    @property
    def max_size(self) -> int:
        """Maximum capacity of the buffer."""
        return self._buffer.maxlen  # type: ignore[return-value]

    # ----- container protocol -----

    def __len__(self) -> int:
        return len(self._buffer)

    def __iter__(self) -> Iterator[Frame]:
        return iter(self._buffer)

    def __getitem__(self, idx: int) -> Frame:
        return list(self._buffer)[idx]

    def __repr__(self) -> str:
        return f"Stream(len={len(self)}, max_size={self.max_size})"

    # ----- mutation -----

    def append(self, frame: Frame) -> None:
        """
        Add one frame to the end of the buffer.

        If the buffer is full, the oldest frame is silently dropped.
        """
        self._buffer.append(frame)

    def extend(self, frames: list[Frame]) -> None:
        """
        Add multiple frames in order.

        Equivalent to calling append() for each frame.
        """
        for f in frames:
            self._buffer.append(f)

    def clear(self) -> None:
        """Remove all frames from the buffer."""
        self._buffer.clear()

    # ----- queries -----

    def latest(self, n: int = 1) -> list[Frame]:
        """
        Return the n most recent frames (newest last).

        Parameters
        ----------
        n : int, default=1
            Number of frames to return. Clamped to buffer length.

        Returns
        -------
        list[Frame]
            Up to n frames, in chronological order (oldest → newest).
        """
        buf = list(self._buffer)
        return buf[-n:]

    def since(self, t: float) -> list[Frame]:
        """
        Return all frames with timestamp >= t.

        Parameters
        ----------
        t : float
            Start time in seconds (monotonic or ROS time).

        Returns
        -------
        list[Frame]
            Frames in chronological order.
        """
        return [f for f in self._buffer if f.timestamp >= t]

    def between(self, t_start: float, t_end: float) -> list[Frame]:
        """
        Return frames whose timestamp falls within [t_start, t_end].

        Parameters
        ----------
        t_start : float
        t_end : float

        Returns
        -------
        list[Frame]
            Frames in chronological order.
        """
        return [f for f in self._buffer if t_start <= f.timestamp <= t_end]

    def to_list(self) -> list[Frame]:
        """Return a plain list copy of all buffered frames (oldest first)."""
        return list(self._buffer)
