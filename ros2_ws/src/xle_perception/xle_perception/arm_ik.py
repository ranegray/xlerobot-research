"""Inverse kinematics for the SO101 left arm via ikpy.

Builds a Chain from the xle_description URDF starting at "Base" and ending
at "Fixed_Jaw" (the gripper mount). The 5 active joints are:

    Rotation_L  Pitch_L  Elbow_L  Wrist_Pitch_L  Wrist_Roll_L

Position-only IK: 5 DOF + 3D position target = 2 redundant DOF, which the
solver consumes by minimizing distance from the initial seed (typically
the current joint state). Orientation is not constrained — the gripper
will end up at the target with whatever orientation the seed steered
toward.

Real EE point is ~5cm forward of "Fixed_Jaw" (between the jaws). This
module does not account for that offset; callers should subtract it from
the target if they want the jaw tip at the target.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
from ikpy.chain import Chain


ARM_JOINTS = ["Rotation_L", "Pitch_L", "Elbow_L", "Wrist_Pitch_L", "Wrist_Roll_L"]
BASE_LINK = "Base"
EE_LINK = "Fixed_Jaw"

# Full link/joint path; avoids the Base -> Base_geom_1 side branch.
_CHAIN_ELEMENTS = [
    "Base",
    "Rotation_L",
    "Rotation_Pitch",
    "Pitch_L",
    "Upper_Arm",
    "Elbow_L",
    "Lower_Arm",
    "Wrist_Pitch_L",
    "Wrist_Pitch_Roll",
    "Wrist_Roll_L",
    "Fixed_Jaw",
]


@dataclass
class IKResult:
    joint_positions: Dict[str, float]
    achieved_xyz: np.ndarray   # FK of the solution, in Base frame
    target_xyz: np.ndarray
    error_m: float


class ArmIK:
    def __init__(self, urdf_path: Path):
        self.chain = Chain.from_urdf_file(
            str(urdf_path),
            base_elements=_CHAIN_ELEMENTS,
            base_element_type="link",
            name="left_arm",
        )
        self._joint_to_chain_index: Dict[str, int] = {}
        active_mask: List[bool] = []
        for i, link in enumerate(self.chain.links):
            joint_name = getattr(link, "name", None) or ""
            if joint_name in ARM_JOINTS:
                self._joint_to_chain_index[joint_name] = i
                active_mask.append(True)
            else:
                # Fixed links are passive.
                active_mask.append(False)
        if set(self._joint_to_chain_index) != set(ARM_JOINTS):
            missing = set(ARM_JOINTS) - set(self._joint_to_chain_index)
            extra = set(self._joint_to_chain_index) - set(ARM_JOINTS)
            raise RuntimeError(
                f"ikpy chain doesn't expose expected arm joints (missing={missing}, "
                f"extra={extra}). Inspect the URDF chain Base -> {EE_LINK}."
            )
        self.chain.active_links_mask = active_mask

    def _seed_vector(self, current: Dict[str, float]) -> np.ndarray:
        v = np.zeros(len(self.chain.links))
        for joint, idx in self._joint_to_chain_index.items():
            q = float(current.get(joint, 0.0))
            # scipy rejects seeds even slightly outside bounds; encoders can
            # land there after calibration offset or overshoot.
            lo, hi = self.chain.links[idx].bounds
            if np.isfinite(lo) and np.isfinite(hi):
                eps = 1e-6 * max(1.0, hi - lo)
                q = min(max(q, lo + eps), hi - eps)
            v[idx] = q
        return v

    def _to_joint_dict(self, full_solution: np.ndarray) -> Dict[str, float]:
        return {
            joint: float(full_solution[idx])
            for joint, idx in self._joint_to_chain_index.items()
        }

    def solve(
        self,
        target_xyz: Sequence[float],
        current_q: Optional[Dict[str, float]] = None,
    ) -> IKResult:
        target = np.asarray(target_xyz, dtype=float)
        seed = self._seed_vector(current_q or {})
        full = self.chain.inverse_kinematics(target_position=target, initial_position=seed)
        achieved = self.chain.forward_kinematics(full)[:3, 3]
        return IKResult(
            joint_positions=self._to_joint_dict(full),
            achieved_xyz=achieved,
            target_xyz=target,
            error_m=float(np.linalg.norm(achieved - target)),
        )

    def fk(self, current_q: Dict[str, float]) -> np.ndarray:
        """Forward kinematics: return EE position in Base frame for the given q."""
        full = self._seed_vector(current_q)
        return self.chain.forward_kinematics(full)[:3, 3]
