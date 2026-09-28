# Project ATHLETE
#
# Generates the AthleteLab level from scratch so the lab is reproducible and reviewable as code.
# Run it with Scripts/BuildLab.ps1 (headless), or inside the editor:
#   Tools > Execute Python Script... > pick this file
#
# The level is a generated artifact: if you hand-edit AthleteLab and re-run this script,
# your edits are lost. Put permanent lab changes here instead.
#
# Layout (Unreal axes: +X forward, +Y right, +Z up; 1 Unreal unit = 1 cm):
#   - 120 m x 60 m flat floor, top surface at Z = 0
#   - measurement lines across the X axis every 5 m from 0 m to 50 m, labeled
#   - sun, sky light, sky atmosphere
#   - a PlayerStart looking down the measurement lane
#   - one physics probe released 3 m above the floor

import unreal

MAP_PATH = "/Game/Lab/Maps/AthleteLab"

CM_PER_M = 100.0
FLOOR_LENGTH_M = 120.0
FLOOR_WIDTH_M = 60.0
FLOOR_THICKNESS_M = 0.1
MARKER_SPACING_M = 5.0
MARKER_MAX_M = 50.0
MARKER_LENGTH_M = 10.0
MARKER_WIDTH_M = 0.05
MARKER_HEIGHT_M = 0.01
PROBE_RELEASE_HEIGHT_M = 3.0

CUBE_MESH = "/Engine/BasicShapes/Cube.Cube"          # 1 m cube, pivot at center
FLOOR_MATERIAL = "/Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial"
MARKER_MATERIAL = "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"
PROBE_CLASS = "/Script/ProjectAthlete.AthleteLabPhysicsProbe"


def log(message):
    unreal.log("[BuildAthleteLab] " + message)


def fail(message):
    unreal.log_error("[BuildAthleteLab] " + message)
    raise RuntimeError(message)


def cm(meters):
    return meters * CM_PER_M


def load(path):
    asset = unreal.load_asset(path)
    if asset is None:
        fail("Could not load " + path)
    return asset


def spawn(actor_class, label, folder, location_cm, rotation=None):
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actor = actors.spawn_actor_from_class(actor_class, location_cm, rotation or unreal.Rotator(0.0, 0.0, 0.0))
    if actor is None:
        fail("Failed to spawn " + label)
    actor.set_actor_label(label)
    actor.set_folder_path(folder)
    return actor


def spawn_box(label, folder, center_m, size_m, material, collide):
    """A cube mesh scaled to size_m (x, y, z in meters)."""
    actor = spawn(unreal.StaticMeshActor, label, folder,
                  unreal.Vector(cm(center_m[0]), cm(center_m[1]), cm(center_m[2])))
    component = actor.static_mesh_component
    component.set_static_mesh(load(CUBE_MESH))
    component.set_material(0, load(material))
    # The engine cube is 1 m on a side, so scale == size in meters.
    actor.set_actor_scale3d(unreal.Vector(size_m[0], size_m[1], size_m[2]))
    if not collide:
        component.set_collision_profile_name("NoCollision")
    return actor


def main():
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        fail(MAP_PATH + " already exists. Delete it in the Content Browser first if you want to regenerate it.")

    level_editor = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not level_editor.new_level(MAP_PATH):
        fail("Could not create level " + MAP_PATH)
    log("Created " + MAP_PATH)

    # Floor: top surface exactly at Z = 0 so heights read directly as meters above ground.
    spawn_box("Floor", "Lab/Ground",
              (FLOOR_LENGTH_M * 0.25, 0.0, -FLOOR_THICKNESS_M * 0.5),
              (FLOOR_LENGTH_M, FLOOR_WIDTH_M, FLOOR_THICKNESS_M),
              FLOOR_MATERIAL, collide=True)

    # Measurement lines. No collision: they are paint, not obstacles.
    distance = 0.0
    while distance <= MARKER_MAX_M + 1e-6:
        spawn_box("Line_%02dm" % int(distance), "Lab/Markings",
                  (distance, 0.0, MARKER_HEIGHT_M * 0.5),
                  (MARKER_WIDTH_M, MARKER_LENGTH_M, MARKER_HEIGHT_M),
                  MARKER_MATERIAL, collide=False)
        text = spawn(unreal.TextRenderActor, "Label_%02dm" % int(distance), "Lab/Markings",
                     unreal.Vector(cm(distance), cm(-MARKER_LENGTH_M * 0.5 - 0.5), 1.0),
                     unreal.Rotator(roll=0.0, pitch=90.0, yaw=180.0))
        text.text_render.set_text("%d m" % int(distance))
        text.text_render.set_world_size(60.0)
        distance += MARKER_SPACING_M

    # Lighting: a sun, sky atmosphere, and a sky light that captures the sky in real time.
    sun = spawn(unreal.DirectionalLight, "Sun", "Lab/Lighting", unreal.Vector(0.0, 0.0, 1000.0),
                unreal.Rotator(roll=0.0, pitch=-50.0, yaw=35.0))
    sun.light_component.set_intensity(8.0)
    spawn(unreal.SkyAtmosphere, "SkyAtmosphere", "Lab/Lighting", unreal.Vector(0.0, 0.0, 0.0))
    sky = spawn(unreal.SkyLight, "SkyLight", "Lab/Lighting", unreal.Vector(0.0, 0.0, 500.0))
    sky.light_component.set_editor_property("real_time_capture", True)

    # Where the spectator camera appears when you press Play: behind and above the start line.
    spawn(unreal.PlayerStart, "PlayerStart", "Lab", unreal.Vector(cm(-8.0), cm(-8.0), cm(3.0)),
          unreal.Rotator(roll=0.0, pitch=0.0, yaw=35.0))

    probe_class = unreal.load_class(None, PROBE_CLASS)
    if probe_class is None:
        fail("Could not find " + PROBE_CLASS + ". Is the C++ project compiled?")
    spawn(probe_class, "PhysicsProbe", "Lab/Instruments", unreal.Vector(0.0, 0.0, cm(PROBE_RELEASE_HEIGHT_M)))

    if not level_editor.save_current_level():
        fail("Could not save " + MAP_PATH)
    log("Saved " + MAP_PATH)


main()
