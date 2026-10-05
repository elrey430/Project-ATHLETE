"""Project ATHLETE - BVH motion capture files: read, forward kinematics, write.

BVH (Biovision Hierarchy) is the common exchange format of motion capture libraries (100STYLE, CMU
conversions, ...). A file has a HIERARCHY of joints (each with an OFFSET from its parent and CHANNELS) and
MOTION rows (one value per channel per frame). Rotation channels apply in the order listed, about the
joint's parent-aligned axes: R_local = R(c1) @ R(c2) @ R(c3). Position channels (normally only on the root)
replace the joint's offset.

World convention here: MuJoCo's, Z up, metres. BVH files are usually Y up, often in centimetres;
`read(path, scale, y_up)` converts.
"""

import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# Y-up BVH (x right, y up, z forward) -> Z-up world: a proper rotation (x, y, z) -> (z, x, y).
Y_UP_TO_Z_UP = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class Joint:
    def __init__(self, name, parent, offset, channels):
        self.name, self.parent, self.offset, self.channels = name, parent, np.asarray(offset, float), channels
        self.column = 0  # first motion column of this joint's channels


class Motion:
    """A parsed BVH file. joints: list in file order (parents before children); values: (frames, channels)."""

    def __init__(self, joints, values, frame_time):
        self.joints, self.values, self.frame_time = joints, values, frame_time
        self.index = {j.name: i for i, j in enumerate(joints)}

    @property
    def frequency(self):
        return 1.0 / self.frame_time

    @property
    def n_frames(self):
        return len(self.values)

    def forward_kinematics(self, frames=None):
        """World positions (frames, joints, 3) and rotations (frames, joints, 3, 3), in the file's own axes
        and units."""
        values = self.values if frames is None else self.values[frames]
        n, count = len(values), len(self.joints)
        positions, rotations = np.zeros((n, count, 3)), np.zeros((n, count, 3, 3))
        for i, joint in enumerate(self.joints):
            local_position = np.tile(joint.offset, (n, 1))
            local_rotation = np.tile(np.eye(3), (n, 1, 1))
            axes, angles = "", []
            for k, channel in enumerate(joint.channels):
                column = values[:, joint.column + k]
                if channel.endswith("position"):
                    local_position[:, "xyz".index(channel[0].lower())] = column
                else:
                    axes += channel[0].upper()  # upper case = intrinsic rotations, applied in listed order
                    angles.append(column)
            if axes:
                local_rotation = Rotation.from_euler(axes, np.stack(angles, axis=1), degrees=True).as_matrix()
            if joint.parent is None:
                positions[:, i], rotations[:, i] = local_position, local_rotation
            else:
                p = joint.parent
                positions[:, i] = positions[:, p] + np.einsum("nij,nj->ni", rotations[:, p], local_position)
                rotations[:, i] = rotations[:, p] @ local_rotation
        return positions, rotations


def parse(text):
    tokens = re.findall(r"\S+", text)
    joints, stack, pos, pending = [], [], 0, None
    column = 0

    def take():
        nonlocal pos
        pos += 1
        return tokens[pos - 1]

    assert take().upper() == "HIERARCHY"
    while True:
        token = take()
        upper = token.upper()
        if upper in ("ROOT", "JOINT"):
            pending = take()
        elif upper == "END":
            take()  # "Site"
            pending = f"{joints[stack[-1]].name}_end"
        elif token == "{":
            stack.append(len(joints))
            joints.append(Joint(pending, stack[-2] if len(stack) > 1 else None, [0.0, 0.0, 0.0], []))
        elif upper == "OFFSET":
            joints[stack[-1]].offset = np.array([float(take()) for _ in range(3)])
        elif upper == "CHANNELS":
            joint = joints[stack[-1]]
            joint.channels = [take() for _ in range(int(take()))]
            joint.column, column = column, column + len(joint.channels)
        elif token == "}":
            stack.pop()
        elif upper == "MOTION":
            break
    assert take().upper().startswith("FRAMES")
    n_frames = int(take())
    assert take().upper() == "FRAME" and take().upper().startswith("TIME")
    frame_time = float(take())
    numbers = np.array(tokens[pos:pos + n_frames * column], float)
    assert numbers.size == n_frames * column, f"expected {n_frames} x {column} motion values, got {numbers.size}"
    return Motion(joints, numbers.reshape(n_frames, column), frame_time)


def read(path):
    return parse(Path(path).read_text(encoding="utf-8", errors="replace"))


def world(points, scale=0.01, y_up=True):
    """File positions (..., 3) -> MuJoCo world (Z up, metres). scale: metres per file unit."""
    points = np.asarray(points) * scale
    return points @ Y_UP_TO_Z_UP.T if y_up else points


def world_rotation(rotations, y_up=True):
    """File rotations (..., 3, 3) -> MuJoCo world axes."""
    return Y_UP_TO_Z_UP @ rotations @ Y_UP_TO_Z_UP.T if y_up else rotations


def write(path, names, parents, offsets, root_positions, local_rotations, frame_time):
    """
    A BVH file (Z up as given, no axis conversion; units as given). names/parents/offsets per joint
    (parents[i] < i, root first); root_positions (frames, 3); local_rotations (frames, joints, 3, 3)
    relative to the parent. Rotations are written as ZYX intrinsic Euler angles.
    """
    children = {i: [k for k, p in enumerate(parents) if p == i] for i in range(len(names))}
    lines, order = ["HIERARCHY"], []  # order: joints as the hierarchy lists them (= motion column order)

    def emit(i, depth):
        order.append(i)
        pad = "  " * depth
        lines.append(f"{pad}{'ROOT' if parents[i] is None else 'JOINT'} {names[i]}")
        lines.append(f"{pad}{{")
        lines.append(f"{pad}  OFFSET {' '.join(f'{v:.6f}' for v in offsets[i])}")
        channels = "Xposition Yposition Zposition Zrotation Yrotation Xrotation" if parents[i] is None \
            else "Zrotation Yrotation Xrotation"
        lines.append(f"{pad}  CHANNELS {len(channels.split())} {channels}")
        for child in children[i]:
            emit(child, depth + 1)
        if not children[i]:
            lines.extend([f"{pad}  End Site", f"{pad}  {{", f"{pad}    OFFSET 0 0 0", f"{pad}  }}"])
        lines.append(f"{pad}}}")

    emit(0, 0)
    n = len(root_positions)
    euler = Rotation.from_matrix(local_rotations.reshape(-1, 3, 3)).as_euler("ZYX", degrees=True).reshape(n, -1, 3)
    lines += ["MOTION", f"Frames: {n}", f"Frame Time: {frame_time:.8f}"]
    for f in range(n):
        lines.append(" ".join(f"{v:.6f}" for v in np.concatenate([root_positions[f], euler[f, order].ravel()])))
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
