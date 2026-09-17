"""Dependency and architecture check for the Python 3.13 receiver."""
from __future__ import annotations

import platform
import struct
import sys


def fail(message: str) -> None:
    print(f"[FAILED] {message}")
    raise SystemExit(1)


print("MaixCAM2 receiver environment check")
print(f"Python    : {sys.version.split()[0]}")
print(f"Executable: {sys.executable}")
print(f"Platform  : {platform.platform()}")
print(f"Bits      : {struct.calcsize('P') * 8}")

if sys.version_info[:2] != (3, 13):
    fail("This package expects CPython 3.13.")
if struct.calcsize("P") * 8 != 64:
    fail("64-bit Python is required.")

try:
    import numpy
    import av
    import PySide6
except Exception as exc:
    fail(f"Dependency import error: {exc}")

print(f"NumPy     : {numpy.__version__}")
print(f"PyAV      : {av.__version__}")
print(f"PySide6   : {PySide6.__version__}")

output_type = av.container.OutputContainer
if not (
    hasattr(output_type, "add_stream_from_template")
    or hasattr(output_type, "add_stream")
):
    fail("The installed PyAV build does not expose a supported remux API.")

print("[OK] Python 3.13 environment is ready.")
