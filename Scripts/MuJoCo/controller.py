"""Project ATHLETE - the controller layer: what the player asks for, shaped into what an athlete can do.

A stick can jump from "stand" to "full sprint", or from sprinting to a sidestep, between two frames. A
person can't: forward speed, sideways speed and turning rate change no faster than the body's acceleration
allows. CommandShaper limits how fast each part of the command changes, before motion matching turns it
into a reference motion. Measured 2026-10-03: most falls in the random-command test came right after such
instant jumps (stand -> 2.9 m/s, sprint -> stop, sprint -> sidestep).

Limits (ESTIMATES of human capability, to be tuned for game feel):
  - forward acceleration 3.5 m/s^2: well inside a sprinter's start (~4-6 m/s^2 over the first steps);
  - forward deceleration 4.0 m/s^2: a hard stop from a run;
  - sideways acceleration 2.5 m/s^2;
  - turning-rate change 4.0 rad/s^2.
The shaped command reaches the acceptance tests' targets well within their time limits
(e.g. 0 -> 1.5 m/s in 0.43 s, 1.5 -> 0 m/s in 0.38 s, 0 -> 0.8 rad/s in 0.2 s).
"""

import numpy as np

FORWARD_ACCELERATION = 3.5   # m/s^2, ESTIMATE
FORWARD_DECELERATION = 4.0   # m/s^2, ESTIMATE
SIDEWAYS_ACCELERATION = 2.5  # m/s^2, ESTIMATE
TURN_ACCELERATION = 4.0      # rad/s^2, ESTIMATE


class CommandShaper:
    """Rate-limits a (forward m/s, sideways m/s, turning rad/s) command toward what the player asks for."""

    def __init__(self):
        self.reset()

    def reset(self, command=(0.0, 0.0, 0.0)):
        self.command = np.asarray(command, float).copy()
        return tuple(self.command)

    def step(self, wanted, dt):
        wanted = np.asarray(wanted, float)
        forward, sideways, turn = self.command
        # Forward speed: speeding up (away from zero) is limited by acceleration, slowing down by deceleration.
        speeding_up = abs(wanted[0]) > abs(forward) and np.sign(wanted[0]) == np.sign(forward or wanted[0])
        limit = (FORWARD_ACCELERATION if speeding_up else FORWARD_DECELERATION) * dt
        forward += np.clip(wanted[0] - forward, -limit, limit)
        sideways += np.clip(wanted[1] - sideways, -SIDEWAYS_ACCELERATION * dt, SIDEWAYS_ACCELERATION * dt)
        turn += np.clip(wanted[2] - turn, -TURN_ACCELERATION * dt, TURN_ACCELERATION * dt)
        self.command = np.array([forward, sideways, turn])
        return tuple(self.command)
