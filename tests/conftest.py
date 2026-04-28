from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XLE_HARDWARE_SRC = ROOT / "ros2_ws" / "src" / "xle_hardware"

sys.path.insert(0, str(XLE_HARDWARE_SRC))
