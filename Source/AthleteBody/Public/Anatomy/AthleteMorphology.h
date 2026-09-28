// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteSegments.h"
#include "AthleteMorphology.generated.h"

/**
 * The inputs that define an athlete's body: size, proportions, and mass distribution.
 *
 * Everything else (segment lengths, masses, centers of mass, moments of inertia) is COMPUTED
 * from these values by FAthleteBodyModel. Nothing here is a rating, and nothing here grants
 * a gameplay bonus: a player's agility or tackle-breaking must emerge from what these values
 * do inside the physics.
 *
 * All proportion and mass scales are dimensionless multipliers relative to the de Leva (1996)
 * reference male (1.0 = reference proportions).
 *
 * USTRUCT + GENERATED_BODY make this struct part of Unreal's reflection system, so it can be
 * stored in data assets, edited in the Details panel, saved, and used in Blueprints.
 * UPROPERTY meta "Units" makes the editor show units (and convert to your preferred display
 * units, e.g. feet and pounds, per Editor Preferences > General > Appearance > Units).
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteMorphology
{
	GENERATED_BODY()

	// ---- Size ----

	/** Standing height, barefoot. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Size", meta = (Units = "m", ClampMin = "1.2", ClampMax = "2.4"))
	double StatureM = 1.741;

	/** Total body mass. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Size", meta = (Units = "kg", ClampMin = "40", ClampMax = "220"))
	double BodyMassKg = 73.0;

	/** Age. Used only by population-baseline strength estimates, never as a gameplay modifier. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Size", meta = (Units = "yr", ClampMin = "15", ClampMax = "60"))
	double AgeYears = 24.0;

	// ---- Proportions (1.0 = reference) ----

	/** Thigh + shank length. Stature is preserved: longer legs mean a shorter head-and-trunk. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proportions", meta = (ClampMin = "0.85", ClampMax = "1.15"))
	double LegLengthScale = 1.0;

	/** Upper arm + forearm + hand length. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proportions", meta = (ClampMin = "0.85", ClampMax = "1.2"))
	double ArmLengthScale = 1.0;

	/** Heel-to-toe foot length. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proportions", meta = (ClampMin = "0.85", ClampMax = "1.2"))
	double FootLengthScale = 1.0;

	/** Distance between the shoulder joint centers. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proportions", meta = (ClampMin = "0.8", ClampMax = "1.3"))
	double ShoulderWidthScale = 1.0;

	/** Distance between the hip joint centers. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proportions", meta = (ClampMin = "0.8", ClampMax = "1.3"))
	double HipWidthScale = 1.0;

	// ---- Mass distribution (relative weights, renormalized to BodyMassKg) ----

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass Distribution", meta = (ClampMin = "0.5", ClampMax = "2.0"))
	double HeadNeckMassScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass Distribution", meta = (ClampMin = "0.5", ClampMax = "2.0"))
	double TrunkMassScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass Distribution", meta = (ClampMin = "0.5", ClampMax = "2.0"))
	double ArmMassScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass Distribution", meta = (ClampMin = "0.5", ClampMax = "2.0"))
	double LegMassScale = 1.0;

	double GetRegionMassScale(EAthleteBodyRegion Region) const;

	/** Returns false (with a human-readable reason) if the values cannot describe a body. */
	bool Validate(FString* OutError = nullptr) const;

	/** Convenience for football-style data entry: 5 ft 9 in, 190 lb. */
	static FAthleteMorphology FromImperial(double Feet, double Inches, double Pounds, double InAgeYears = 22.0);
};
