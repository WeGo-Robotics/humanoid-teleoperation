"""Forward kinematics for the arm IK models.

The align gate needs to answer "where are the robot's wrists right now, in the
same frame the operator's wrist targets are expressed in". The IK classes
already carry a pinocchio reduced model with `L_ee` / `R_ee` frames -- the exact
frames `solve_ik` drives to targets -- so FK through that model is guaranteed to
be in the right frame by construction. Deriving it any other way would risk the
comparison being done in two subtly different frames, which is precisely the
class of bug the gate exists to catch.
"""
import numpy as np


class ArmFKMixin:
    """Mixed into every `*_ArmIK`. Requires `reduced_robot`, `L_hand_id`,
    `R_hand_id` -- all five classes already define them."""

    def forward_kinematics(self, q):
        """Wrist poses for joint configuration `q`.

        Returns (left, right) as 4x4 matrices in the IK target frame, or
        (None, None) if FK could not be evaluated -- the caller must treat that
        as "cannot verify alignment", never as "aligned".
        """
        try:
            import pinocchio as pin
            model = self.reduced_robot.model
            data = self.reduced_robot.data
            q = np.asarray(q, dtype=float).reshape(-1)
            if q.shape[0] != model.nq:
                raise ValueError(
                    f"expected {model.nq} joint values, got {q.shape[0]}")
            pin.framesForwardKinematics(model, data, q)
            return (np.array(data.oMf[self.L_hand_id].homogeneous),
                    np.array(data.oMf[self.R_hand_id].homogeneous))
        except Exception as e:
            try:
                from logging_mp import get_logger
                get_logger(__name__).error(f"[ArmFK] forward_kinematics failed: {e}")
            except Exception:
                pass
            return None, None
