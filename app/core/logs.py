"""The app's log (2026-11-11; until then each module print()ed). Every module logs through
`logging.getLogger(__name__)`; setup() sends the "app" loggers to stdout, which start_lidar.sh
keeps in logs/run_<date>_<time>.log, one line each:

    10:24:16.512 radar_utils: [RADAR] deg030 streaming
    10:24:17.004 WARNING pi_camera_utils: Camera: no frame for 3s — the camera is not delivering

The level is shown only from WARNING up. SHAPPE_LOG_LEVEL (DEBUG, INFO, WARNING, ERROR;
default INFO) sets how much is written. What stays print(): the live guidance console
(test_console), the start-up banner, and the hardware checks run by hand (--check)."""

import logging
import os
import sys
from typing import Optional

_FORMAT = "%(asctime)s.%(msecs)03d %(levelflag)s%(module)s: %(message)s"


class _Stdout(logging.StreamHandler):
    """Writes to whatever sys.stdout is when a line is logged, not the one at setup: a test
    that redirects stdout sees the lines, as it saw the prints."""

    def __init__(self) -> None:
        super().__init__(sys.stdout)

    @property  # type: ignore[override]
    def stream(self):
        return sys.stdout

    @stream.setter
    def stream(self, _value) -> None:
        pass


class _Formatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        record.levelflag = "" if record.levelno < logging.WARNING else f"{record.levelname} "
        return super().format(record)


def setup(level: Optional[str] = None) -> None:
    """Send the app's log to stdout. Idempotent; the libraries' loggers are left alone."""
    root = logging.getLogger("app")
    if not any(isinstance(h, _Stdout) for h in root.handlers):
        handler = _Stdout()
        handler.setFormatter(_Formatter(_FORMAT, "%H:%M:%S"))
        root.addHandler(handler)
    name = (level or os.environ.get("SHAPPE_LOG_LEVEL") or "INFO").upper()
    root.setLevel(getattr(logging, name, logging.INFO))
    root.propagate = False
