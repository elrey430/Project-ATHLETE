"""Project ATHLETE - the athlete as a LocoMuJoCo environment (imitation learning on GPUs).

Import this package before creating environments: it registers AthleteReference (CPU MuJoCo) and
MjxAthleteReference (GPU: MJX / MuJoCo Warp) with LocoMuJoCo.
"""

from .commands import GoalVelocityCommand, VelocityCommandReward
from .env import AthleteReference, MjxAthleteReference
from .invariant import GoalTrackInvariant
from .lookahead import GoalTrajMimicLookahead
from .rewards import MimicHeadingReward

AthleteReference.register()
MjxAthleteReference.register()
GoalVelocityCommand.register()
VelocityCommandReward.register()
GoalTrajMimicLookahead.register()
MimicHeadingReward.register()
GoalTrackInvariant.register()
