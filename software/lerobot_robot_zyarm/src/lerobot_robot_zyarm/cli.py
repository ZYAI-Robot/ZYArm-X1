from __future__ import annotations

import sys


def _configure_console_encoding() -> None:
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            reconfigure(encoding="utf-8", errors="replace")


def main() -> None:
    _configure_console_encoding()
    from .recording import main as record_main

    record_main()


__all__ = ["main"]
