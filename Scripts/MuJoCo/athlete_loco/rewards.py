"""Project ATHLETE - tracking reward with an explicit heading term.

LocoMuJoCo's MimicReward (DeepMimic) compares the whole pose; the pelvis's facing is one quaternion in its
joint-position term, next to 37 joint angles, and every other term is relative to the pelvis, so blind to
facing. The tracker learned walking, running and braking with it, but not turning: it ignored the
reference's heading even when nothing held it back (2026-10-01).

MimicHeadingReward adds terms to MimicReward:
  heading_w_sum * exp(-heading_w_exp * e_yaw^2)          e_yaw: facing error (rad, wrapped)
  yaw_rate_w_sum * exp(-yaw_rate_w_exp * e_rate^2)       e_rate: turning-rate error (rad/s)
  velocity_w_sum * exp(-velocity_w_exp * |e_v|^2)        e_v: pelvis forward/sideways speed error (m/s),
                                                          each speed in its own body's heading frame
The velocity term (default off) came after the heading-invariant tracker turned well but lagged the
reference speed, ~0.2 m/s walking and ~1 m/s running (2026-10-02).
"""

from typing import Any, Dict, Tuple

from loco_mujoco.core.reward.trajectory_based import MimicReward


def _heading(quat, backend):
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    return backend.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


class MimicHeadingReward(MimicReward):

    def __init__(self, env: Any, heading_w_sum: float = 0.3, heading_w_exp: float = 5.0,
                 yaw_rate_w_sum: float = 0.2, yaw_rate_w_exp: float = 1.0,
                 velocity_w_sum: float = 0.0, velocity_w_exp: float = 2.0, **kwargs):
        super().__init__(env, **kwargs)
        self._velocity_w_sum, self._velocity_w_exp = velocity_w_sum, velocity_w_exp
        self._heading_w_sum, self._heading_w_exp = heading_w_sum, heading_w_exp
        self._yaw_rate_w_sum, self._yaw_rate_w_exp = yaw_rate_w_sum, yaw_rate_w_exp

    def __call__(self, state, action, next_state, absorbing: bool, info: Dict[str, Any], env: Any, model, data,
                 carry: Any, backend) -> Tuple[float, Any]:
        reward, carry = super().__call__(state, action, next_state, absorbing, info, env, model, data, carry, backend)
        reference = env.th.traj.data.get(carry.traj_state.traj_no, carry.traj_state.subtraj_step_no, backend)
        yaw_sim, yaw_ref = _heading(data.qpos[3:7], backend), _heading(reference.qpos[3:7], backend)
        error = yaw_sim - yaw_ref
        error = backend.arctan2(backend.sin(error), backend.cos(error))  # wrapped to [-pi, pi]
        rate_error = data.qvel[5] - reference.qvel[5]  # turning rate: the pelvis's angular velocity about its z
        def planar(velocity, yaw):  # world xy velocity in a heading frame (forward, sideways)
            c, s = backend.cos(yaw), backend.sin(yaw)
            return backend.array([c * velocity[0] + s * velocity[1], -s * velocity[0] + c * velocity[1]])
        speed_error = planar(data.qvel[:2], yaw_sim) - planar(reference.qvel[:2], yaw_ref)
        reward = (reward + self._heading_w_sum * backend.exp(-self._heading_w_exp * error ** 2)
                  + self._yaw_rate_w_sum * backend.exp(-self._yaw_rate_w_exp * rate_error ** 2)
                  + self._velocity_w_sum * backend.exp(-self._velocity_w_exp * backend.sum(speed_error ** 2)))
        return reward, carry
