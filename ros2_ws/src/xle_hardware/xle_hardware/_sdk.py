"""Import the Feetech SCS servo SDK with a friendly install hint on failure.

scservo_sdk ships as the PyPI package `feetech-servo-sdk`. It is not packaged
for apt or rosdep, so it must be pip-installed into the python ROS uses.
"""

from __future__ import annotations

import sys

_INSTALL_HINT = (
    "scservo_sdk is not installed. Install with:\n"
    "    python3 -m pip install --user 'feetech-servo-sdk>=1.0.0,<2.0.0'\n"
    "Then re-run this script."
)


def load_sdk():
    try:
        import scservo_sdk
    except ImportError:
        print(_INSTALL_HINT, file=sys.stderr)
        raise SystemExit(2)
    return scservo_sdk
