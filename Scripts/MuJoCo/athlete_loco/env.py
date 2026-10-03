"""Project ATHLETE - LocoMuJoCo environments for the exported athlete.

The body is the MuJoCo model Unreal exports (AthleteMjcfExport, Saved/MuJoCo/*.xml): de Leva segments,
anatomical hinges with their ranges, one torque motor per hinge capped at isometric strength, and the
15 "*_mimic" marker sites LocoMuJoCo's imitation rewards compare. Nothing here changes the body.

Model file: $ATHLETE_MJCF, else Saved/MuJoCo/athlete_reference.xml in this project.

Changes made here, for training AND testing alike (configure_contacts): the same body must be
trained and run (2026-10-01: running learned on the GPU's simplified feet didn't transfer to the exported
box feet).
  - Feet: a heel and a forefoot capsule per foot, the forefoot one at the ball of the foot, where a rigid
    foot without toes rolls over (a box rolls over its toe tip). The exported box stays for display.
  - Contacts: feet with the floor and each other, capsule limbs with the floor, plus self-collision
    between capsule limbs (legs with each other, forearms and hands with each other).

Known gaps:
  - LocoMuJoCo's DefaultControl maps actions to torques within the motors' limits (isometric strength);
    the muscles' force-velocity (Hill) limits aren't applied yet.
  - Passive joint damping and armature are added here. The armature (JOINT_ARMATURE_KG_M2) is a numerical
    compromise, not anatomy.
  - No arm-trunk or arm-leg collision yet: the trunk segments are boxes, which MuJoCo Warp collides through
    a buffer that overflowed (see configure_contacts); they need capsule stand-ins.
  - The C++ exporter and Unreal still have box feet: move this foot model there too.
"""

import os
from pathlib import Path
from typing import List, Tuple, Union

import mujoco
import numpy as np
from mujoco import MjSpec

from loco_mujoco.core import Observation, ObservationType
from loco_mujoco.core.utils import info_property
from loco_mujoco.environments.humanoids.base_robot_humanoid import BaseRobotHumanoid

PROJECT_ROOT = Path(__file__).resolve().parents[3]
FOOT_GEOMS = ["FootLeft", "FootRight"]

# Passive joint properties added for MuJoCo (2026-09-30). Without them, random torques of an untrained
# policy made the simulation blow up (NaN) within 0.04-0.6 s in 47-64 of 64 worlds, in float64 too.
# Cause: each anatomical joint is 3 hinges in a row; when the middle one reaches +/-90 deg (e.g. the arm
# overhead) the outer two line up (gimbal lock), the joint-space mass matrix turns singular and velocities
# explode. Measured fixes (32 worlds, 3 s, random actions up to full strength): damping + armature: 0 NaN;
# damping alone: 11/32. The same values as LocoMuJoCo's human skeleton (SkeletonTorque), trained to walk.
#  - DAMPING is physical: joint tissue's viscous resistance. ESTIMATE in the physiological range.
#  - ARMATURE is NOT physical: extra inertia on every hinge that keeps the 3-hinge parametrization solvable
#    through gimbal lock. A modelling compromise (median ~3% of a hinge's real inertia, far more on the
#    lightest hinges); ball joints for the 3-axis joints would remove it.
JOINT_DAMPING_NMS_PER_RAD = 1.0
JOINT_ARMATURE_KG_M2 = 0.01


def apply_passive_joint_properties(spec: MjSpec) -> MjSpec:
    for joint in spec.joints:
        if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            joint.damping = JOINT_DAMPING_NMS_PER_RAD
            joint.armature = JOINT_ARMATURE_KG_M2
    return spec


# Self-collision between capsule limbs (exact, cheap contact tests). Pairs that overlap in the exported
# reference pose (thigh-thigh, forearm/hand-same thigh) are excluded by the export and left out here.
SELF_COLLISION_PAIRS = [("ShankLeft", "ShankRight"), ("ShankLeft", "ThighRight"), ("ThighLeft", "ShankRight"),
                        ("ForearmLeft", "ForearmRight"), ("HandLeft", "HandRight"),
                        ("HandLeft", "ForearmRight"), ("ForearmLeft", "HandRight")]
SELF_COLLISION_FRICTION = 0.3  # ESTIMATE: skin/kit on skin/kit, lower than shoe on ground
# Floor contact for every capsule limb, so a falling athlete lands on knees, hands or head instead of
# passing through the floor (the trunk boxes still do: see Known gaps).
FLOOR_CONTACT_LIMBS = ["Head", "UpperArmLeft", "ForearmLeft", "HandLeft", "UpperArmRight", "ForearmRight",
                       "HandRight", "ThighLeft", "ShankLeft", "ThighRight", "ShankRight"]
SOLE_SITES = {"FootLeft": ("left_heel", "left_ball_of_foot"), "FootRight": ("right_heel", "right_ball_of_foot")}
SHANK_OF_OTHER_FOOT = {"FootLeft": "ShankRight", "FootRight": "ShankLeft"}


def sole_geom_names(foot: str) -> List[str]:
    return [f"{foot}Heel", f"{foot}Ball"]


def configure_contacts(spec: MjSpec) -> MjSpec:
    """
    The athlete's contact model, used for training and testing alike:
     1. Each foot's sole as two capsules across the foot: one at the heel, one centred on the ball of the
        foot (the exported *_ball_of_foot site, 73% of foot length), bottoms on the sole plane. The ball is
        where a rigid foot without toes should roll over at push-off; the previous GPU-only soles had the
        front capsule at the toe tip. The exported box stays for display only.
        Capsules also suit MuJoCo Warp, which collides boxes through a general convex routine whose buffer
        overflowed ("CCD overflow - please increase naccdmax", 13,000+ times on 2026-09-30, foot contacts
        dropped) and can't be raised through MJX or LocoMuJoCo. Capsule contacts are exact.
     2. Only listed contact pairs exist (GPU physics slows down badly with every pair possible: 57 samples/s
        measured): soles-floor, sole-sole, soles against the other leg's shank, SELF_COLLISION_PAIRS, and
        the capsule limbs with the floor (FLOOR_CONTACT_LIMBS).
    """
    boxes = {g.name: g for g in spec.geoms if g.name in FOOT_GEOMS}
    reference = boxes[FOOT_GEOMS[0]]
    sites = {s.name: s for s in spec.sites}
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0

    for name, box in boxes.items():
        _, half_width, half_height = box.size
        radius = half_height  # capsule bottoms on the sole plane
        heel, ball = (np.array(sites[site].pos) for site in SOLE_SITES[name])
        body = next(b for b in spec.bodies if b.name == name)
        for geom_name, x in zip(sole_geom_names(name), (heel[0] + radius, ball[0])):
            body.add_geom(
                name=geom_name, type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                pos=[x, box.pos[1], heel[2] + radius], quat=[1.0, 1.0, 0.0, 0.0],  # axis across the foot (Y)
                size=[radius, half_width - radius, 0.0], rgba=[0.9, 0.9, 0.9, 0.3], contype=0, conaffinity=0)

    def add_pair(first: str, second: str, friction: float) -> None:
        # An explicit pair doesn't inherit the geoms' contact settings: copy the exported stiffness, or
        # MuJoCo's soft defaults would apply.
        pair = spec.add_pair(geomname1=first, geomname2=second)
        pair.condim = 3
        pair.solref = reference.solref
        pair.solimp = reference.solimp
        pair.friction = [friction, friction, reference.friction[1], reference.friction[2], reference.friction[2]]

    ground = reference.friction[0]
    for name in FOOT_GEOMS:
        for capsule in sole_geom_names(name):
            add_pair("floor", capsule, ground)
            add_pair(capsule, SHANK_OF_OTHER_FOOT[name], SELF_COLLISION_FRICTION)
    for left in sole_geom_names(FOOT_GEOMS[0]):
        for right in sole_geom_names(FOOT_GEOMS[1]):
            add_pair(left, right, SELF_COLLISION_FRICTION)
    for first, second in SELF_COLLISION_PAIRS:
        add_pair(first, second, SELF_COLLISION_FRICTION)
    for limb in FLOOR_CONTACT_LIMBS:
        add_pair("floor", limb, ground)
    return spec


def build_athlete_model(path: str = None) -> mujoco.MjModel:
    """The athlete as trained and tested (passive joints, contact model), without LocoMuJoCo."""
    spec = mujoco.MjSpec.from_file(path or default_model_path())
    return configure_contacts(apply_passive_joint_properties(spec)).compile()


def lowest_sole_point(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Lowest point of the sole capsules (world z), after kinematics."""
    lowest = np.inf
    for foot in FOOT_GEOMS:
        for name in sole_geom_names(foot):
            g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            axis = data.geom_xmat[g].reshape(3, 3)[:, 2]  # capsule axis: local z
            ends_z = data.geom_xpos[g][2] + np.array([1.0, -1.0]) * axis[2] * model.geom_size[g][1]
            lowest = min(lowest, float(ends_z.min() - model.geom_size[g][0]))
    return lowest


def default_model_path() -> str:
    return os.environ.get("ATHLETE_MJCF", str(PROJECT_ROOT / "Saved" / "MuJoCo" / "athlete_reference.xml"))


def hinge_names(spec: MjSpec) -> List[str]:
    return [j.name for j in spec.joints if j.type == mujoco.mjtJoint.mjJNT_HINGE]


class AthleteReference(BaseRobotHumanoid):
    """
    The athlete on CPU MuJoCo.
    invariant_obs: the observation doesn't depend on which way the athlete faces (invariant.py); for the
    heading-invariant tracker. Default: LocoMuJoCo's humanoid observation (pelvis pose in world axes), which
    the earlier policies were trained with. (An option, not a separate class: LocoMuJoCo finds motion
    data by the environment's class name.)
    """

    mjx_enabled = False

    def __init__(self, spec: Union[str, MjSpec] = None, observation_spec: List[Observation] = None,
                 actuation_spec: List[str] = None, invariant_obs: bool = False, **kwargs) -> None:
        if spec is None:
            spec = self.get_default_xml_file_path()
        spec = mujoco.MjSpec.from_file(spec) if not isinstance(spec, MjSpec) else spec
        if observation_spec is not None:
            observation_spec = self.parse_observation_spec(observation_spec)
        elif invariant_obs:
            from .invariant import invariant_observation_spec
            observation_spec = invariant_observation_spec(hinge_names(spec))
        else:
            observation_spec = self._get_observation_specification(spec)
        if actuation_spec is None:
            actuation_spec = self._get_action_specification(spec)
        spec = configure_contacts(apply_passive_joint_properties(spec))
        if self.mjx_enabled:
            spec = self._modify_spec_for_mjx(spec)
        super().__init__(spec=spec, actuation_spec=actuation_spec, observation_spec=observation_spec, **kwargs)

    @staticmethod
    def _get_observation_specification(spec: MjSpec) -> List[Observation]:
        # Like LocoMuJoCo's humanoids: root pose without x/y, every joint angle, then all velocities.
        hinges = hinge_names(spec)
        return ([ObservationType.FreeJointPosNoXY("q_root", xml_name="root")]
                + [ObservationType.JointPos(f"q_{name}", xml_name=name) for name in hinges]
                + [ObservationType.FreeJointVel("dq_root", xml_name="root")]
                + [ObservationType.JointVel(f"dq_{name}", xml_name=name) for name in hinges])

    @staticmethod
    def _get_action_specification(spec: MjSpec) -> List[str]:
        return [actuator.name for actuator in spec.actuators]

    @classmethod
    def get_default_xml_file_path(cls) -> str:
        return default_model_path()

    @info_property
    def root_body_name(self) -> str:
        return "LowerTrunk"

    @info_property
    def upper_body_xml_name(self) -> str:
        return "UpperTrunk"

    @info_property
    def root_free_joint_xml_name(self) -> str:
        return "root"

    @info_property
    def root_height_healthy_range(self) -> Tuple[float, float]:
        # Height of the root body (pelvis) for which the athlete counts as up; outside it the episode ends
        # (HeightBasedTerminalStateHandler, used by the commanded-locomotion config). Standing ~0.97 m for
        # the reference athlete, running dips to ~0.85; a fallen body is far lower.
        return (0.6, 1.4)

    @info_property
    def foot_geom_names(self) -> List[str]:
        return FOOT_GEOMS

    @info_property
    def goal_visualization_arrow_offset(self) -> List[float]:
        return [0.0, 0.0, 0.6]


class MjxAthleteReference(AthleteReference):
    """The athlete on the GPU (MJX or MuJoCo Warp), set up the way LocoMuJoCo sets up its humanoids."""

    mjx_enabled = True

    def __init__(self, timestep: float = 0.002, n_substeps: int = 5, **kwargs) -> None:
        # Few solver iterations: a GPU batch waits for its slowest world (as MuJoCo's MJX docs advise and
        # LocoMuJoCo's human skeleton uses). No Euler damping: MuJoCo's implicit integrator handles damping.
        model_option_conf = kwargs.pop("model_option_conf", None) or dict(
            iterations=4, ls_iterations=8, disableflags=mujoco.mjtDisableBit.mjDSBL_EULERDAMP)
        super().__init__(timestep=timestep, n_substeps=n_substeps, model_option_conf=model_option_conf, **kwargs)

    def _modify_spec_for_mjx(self, spec: MjSpec) -> MjSpec:
        """GPU specifics only; the contact model is shared with the CPU model (configure_contacts)."""
        # The export turns on energy bookkeeping for the CPU checks; MJX doesn't support it.
        spec.option.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_ENERGY)
        return spec
