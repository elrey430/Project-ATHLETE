"""
Project ATHLETE - reads AddBiomechanics .b3d files (pure Python + NumPy; no nimblephysics).

AddBiomechanics (Stanford; CC BY 4.0; Werling et al., "AddBiomechanics Dataset: Capturing the Physics of
Human Motion at Scale", ECCV 2024, doi:10.1007/978-3-031-73223-2_27) ships one .b3d per subject. The only
official reader, nimblephysics, has no Windows build (2026-11). The layout, from nimblephysics' source (MIT;
dart/proto/SubjectOnDisk.proto, dart/biomechanics/SubjectOnDisk.cpp):
  int64 header size | SubjectOnDiskHeader (protobuf) | frames
Frames are stored trial by trial; each is one SubjectOnDiskSensorFrame (raw_sensor_frame_size bytes) then
one SubjectOnDiskProcessingPassFrame per processing pass (processing_pass_frame_size bytes each), all of
fixed size. A minimal protobuf wire decoder reads them (proto3: repeated numbers are packed).

Units and axes are OpenSim's: metres, seconds, Newtons, Y up. Joint centres are in the order of the model's
joints (Header.joint_names, from the OpenSim model stored in the file).
"""

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PASS_TYPES = ("kinematics", "dynamics", "lowPassFilter", "accMinimizingFilter")
TRIAL_FEATURES = ("walking", "running", "unevenTerrain", "flatTerrain")


def _varint(buf, i):
    result, shift = 0, 0
    while True:
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if b < 0x80:
            return result, i
        shift += 7


def _fields(buf):
    """{field number: [raw values]}: varints as int, fixed64/32 as bytes, length-delimited as bytes."""
    out, i, n = {}, 0, len(buf)
    while i < n:
        key, i = _varint(buf, i)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, i = _varint(buf, i)
        elif wire == 1:
            value, i = buf[i:i + 8], i + 8
        elif wire == 2:
            length, i = _varint(buf, i)
            value, i = buf[i:i + length], i + length
        elif wire == 5:
            value, i = buf[i:i + 4], i + 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
        out.setdefault(number, []).append(value)
    return out


def _doubles(fields, number):
    """A packed repeated double field (or unpacked fixed64 values)."""
    parts = fields.get(number, [])
    return np.frombuffer(b"".join(parts), "<f8") if parts else np.zeros(0)


def _ints(fields, number):
    values = []
    for part in fields.get(number, []):
        if isinstance(part, int):
            values.append(part)
        else:  # packed
            i = 0
            while i < len(part):
                v, i = _varint(part, i)
                values.append(v)
    return values


def _int(fields, number, default=0):
    return fields.get(number, [default])[-1]


def _double(fields, number, default=0.0):
    return struct.unpack("<d", fields[number][-1])[0] if number in fields else default


def _str(fields, number):
    return [bytes(v).decode("utf-8", "replace") for v in fields.get(number, [])]


class Trial:
    def __init__(self, fields):
        self.name = (_str(fields, 1) or [""])[0]
        self.length = _int(fields, 3)
        self.timestep = _double(fields, 4)
        self.num_passes = len(fields.get(5, []))
        self.tags = _str(fields, 6)
        self.original_name = (_str(fields, 10) or [""])[0]
        self.split_index = _int(fields, 11)
        self.missing_grf = _ints(fields, 2)  # per frame: 0 = measured ground force is usable
        self.features = [TRIAL_FEATURES[v] if v < len(TRIAL_FEATURES) else v for v in _ints(fields, 18)]

    def __repr__(self):
        return f"Trial({self.name!r}, {self.length} frames @ {1 / self.timestep if self.timestep else 0:.0f} Hz, tags={self.tags})"


class B3D:
    """One subject. B3D(path).trials, .joint_names, .read(trial) -> dict of per-frame arrays."""

    def __init__(self, path):
        self.path = Path(path)
        with open(self.path, "rb") as f:
            size = struct.unpack("<q", f.read(8))[0]
            header = _fields(f.read(size))
        self.data_start = 8 + size
        self.num_dofs = _int(header, 1)
        self.num_joints = _int(header, 24)
        self.sensor_frame_size = _int(header, 3)
        self.pass_frame_size = _int(header, 4)
        self.passes = []
        for raw in header.get(5, []):
            p = _fields(raw)
            kind = _int(p, 1)
            self.passes.append({"type": PASS_TYPES[kind] if kind < len(PASS_TYPES) else kind, "osim": (_str(p, 2) or [""])[0]})
        self.contact_bodies = _str(header, 6)
        self.trials = [Trial(_fields(raw)) for raw in header.get(9, [])]
        self.version = _int(header, 10)
        self.sex = (_str(header, 13) or [""])[0]
        self.height_m = _double(header, 14)
        self.mass_kg = _double(header, 15)
        self.age = _int(header, 16)
        self.tags = _str(header, 23)
        self.joint_names, self.dof_names = self._model_names(self.passes[0]["osim"] if self.passes else "")
        if len(self.dof_names) != self.num_dofs:
            # The Rajagopal model's kneecap coordinates (knee_angle_*_beta, coupled to the knee) aren't stored.
            self.dof_names = [n for n in self.dof_names if not n.endswith("_beta")]

    @staticmethod
    def _model_names(osim):
        """Joint and coordinate names in the OpenSim model's order (nimblephysics keeps it)."""
        if not osim:
            return [], []
        root = ET.fromstring(osim)
        joints = [j.get("name") for j in root.iter() if j.tag.endswith("Joint") and j.get("name")]
        coordinates = [c.get("name") for c in root.iter("Coordinate") if c.get("name")]
        return joints, coordinates

    def center_names(self, count):
        """Names of the stored joint centres. nimblephysics drops the kneecap joints (patellofemoral) of the
        Rajagopal model: 22 joints, 20 centres (checked by height, Hamner 2013 subject 1)."""
        if len(self.joint_names) == count:
            return list(self.joint_names)
        names = [n for n in self.joint_names if "patellofemoral" not in n]
        if len(names) != count:
            raise ValueError(f"{self.path.name}: {count} joint centres, model joints {self.joint_names}")
        return names

    def read(self, trial, processing_pass=0):
        """Per-frame arrays of one trial from one processing pass (0: kinematics)."""
        t = self.trials[trial]
        frame_size = self.sensor_frame_size + t.num_passes * self.pass_frame_size
        start = self.data_start + sum(x.length * (self.sensor_frame_size + x.num_passes * self.pass_frame_size)
                                      for x in self.trials[:trial])
        with open(self.path, "rb") as f:
            f.seek(start)
            block = f.read(frame_size * t.length)
        names = {1: "pos", 2: "vel", 3: "acc", 4: "tau", 6: "cop", 8: "force", 9: "com_pos", 10: "com_vel",
                 11: "com_acc", 12: "root_residual", 16: "joint_centers"}
        out = {k: [] for k in names.values()}
        offset = self.sensor_frame_size + processing_pass * self.pass_frame_size
        for i in range(t.length):
            frame = _fields(memoryview(block)[i * frame_size + offset:i * frame_size + offset + self.pass_frame_size])
            for number, key in names.items():
                out[key].append(_doubles(frame, number))
        result = {k: np.array(v) for k, v in out.items()}
        result["joint_centers"] = result["joint_centers"].reshape(t.length, -1, 3)
        result["joint_names"] = self.center_names(result["joint_centers"].shape[1])
        n = len(self.contact_bodies)
        result["force"] = result["force"].reshape(t.length, n, 3)  # world, per contact body
        result["cop"] = result["cop"].reshape(t.length, n, 3)
        result["time"] = np.arange(t.length) * t.timestep
        result["grf_ok"] = np.array(t.missing_grf[:t.length]) == 0 if t.missing_grf else np.zeros(t.length, bool)
        return result


if __name__ == "__main__":
    import sys
    for path in sys.argv[1:]:
        s = B3D(path)
        print(f"{s.path.name}: {s.sex}, {s.height_m:.2f} m, {s.mass_kg:.1f} kg, age {s.age}; {s.num_dofs} DOF, {s.num_joints} joints, "
              f"passes {[p['type'] for p in s.passes]}, contact bodies {s.contact_bodies}; {len(s.trials)} trials")
        for k, t in enumerate(s.trials):
            print(f"  {k:2d} {t}  features {t.features}  usable GRF {np.mean(np.array(t.missing_grf) == 0) * 100 if t.missing_grf else 0:.0f}%")
