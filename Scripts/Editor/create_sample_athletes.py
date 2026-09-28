# Project ATHLETE
#
# Creates (or updates in place) the sample athlete Data Assets in /Game/Athletes.
# Safe to re-run: existing assets are updated rather than replaced, so references to them
# (for example from the AthleteLab level) stay intact.
#
# Run headless with Scripts/BuildLab.ps1, or in the editor: Tools > Execute Python Script...

import unreal

ASSET_DIR = "/Game/Athletes"

# Exact definitions (international inch and pound).
METERS_PER_INCH = 0.0254
KILOGRAMS_PER_POUND = 0.45359237

# name, display name, stature (m), mass (kg), age (yr)
ATHLETES = [
    ("DA_Athlete_Reference", "Reference male (de Leva 1996)", 1.741, 73.0, 24.0),
    ("DA_Athlete_A", "Athlete A: 5'9\" 190 lb", (5 * 12 + 9) * METERS_PER_INCH, 190 * KILOGRAMS_PER_POUND, 22.0),
    ("DA_Athlete_B", "Athlete B: 6'4\" 240 lb", (6 * 12 + 4) * METERS_PER_INCH, 240 * KILOGRAMS_PER_POUND, 22.0),
]


def log(message):
    unreal.log("[CreateSampleAthletes] " + message)


def get_or_create(name):
    path = ASSET_DIR + "/" + name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.load_asset(path)
    factory = unreal.DataAssetFactory()
    factory.set_editor_property("data_asset_class", unreal.AthleteDefinition)
    asset = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, ASSET_DIR, unreal.AthleteDefinition, factory)
    if asset is None:
        raise RuntimeError("Could not create " + path)
    log("Created " + path)
    return asset


def main():
    for name, display_name, stature_m, mass_kg, age in ATHLETES:
        asset = get_or_create(name)
        asset.set_editor_property("display_name", unreal.Text(display_name))

        # Structs come back from Python as copies: modify, then write back.
        morphology = asset.get_editor_property("morphology")
        morphology.set_editor_property("stature_m", stature_m)
        morphology.set_editor_property("body_mass_kg", mass_kg)
        morphology.set_editor_property("age_years", age)
        asset.set_editor_property("morphology", morphology)

        # Sourced population baseline (Harbo et al. 2012). Same function as the Details-panel button.
        asset.fill_strength_from_general_population_baseline()

        unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False)
        log("Saved %s/%s: %.4f m, %.2f kg" % (ASSET_DIR, name, stature_m, mass_kg))


main()
