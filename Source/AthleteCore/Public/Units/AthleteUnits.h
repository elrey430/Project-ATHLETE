// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

/**
 * Unit conventions for Project ATHLETE.
 *
 * Three unit systems meet in this project:
 *   1. Unreal Engine world units: centimeters (cm), kilograms (kg), seconds (s).
 *      Gravity in Unreal is -980 cm/s^2. Forces passed to Unreal physics are in kg*cm/s^2.
 *   2. Simulation / biomechanics math: SI units (m, kg, s, N, N*m).
 *      All ATHLETE models, telemetry, and tests are expressed in SI.
 *   3. Football conventions: feet, inches, pounds, yards, miles per hour.
 *      Used only at the edges (athlete definitions, UI) and converted immediately.
 *
 * Rule: convert at the boundary. Simulation code thinks in SI; only the code that
 * talks to Unreal converts to centimeters, and only data entry uses imperial units.
 *
 * All constants are exact by definition (international yard and pound, 1959).
 */
namespace AthleteUnits
{
	// Length
	inline constexpr double CentimetersPerMeter = 100.0;
	inline constexpr double MetersPerInch       = 0.0254;
	inline constexpr double InchesPerFoot       = 12.0;
	inline constexpr double MetersPerFoot       = MetersPerInch * InchesPerFoot;
	inline constexpr double MetersPerYard       = MetersPerFoot * 3.0;
	inline constexpr double MetersPerMile       = MetersPerYard * 1760.0;

	// Mass
	inline constexpr double KilogramsPerPound = 0.45359237;

	// Time
	inline constexpr double SecondsPerHour = 3600.0;

	// Unreal <-> SI length
	[[nodiscard]] constexpr double UnrealToMeters(double Centimeters) { return Centimeters / CentimetersPerMeter; }
	[[nodiscard]] constexpr double MetersToUnreal(double Meters)      { return Meters * CentimetersPerMeter; }

	// Vectors: Unreal cm (or cm/s, cm/s^2) <-> SI m (or m/s, m/s^2). The time unit is unchanged.
	[[nodiscard]] inline FVector UnrealToMeters(const FVector& Centimeters) { return Centimeters / CentimetersPerMeter; }
	[[nodiscard]] inline FVector MetersToUnreal(const FVector& Meters)      { return Meters * CentimetersPerMeter; }

	// Force: Unreal physics uses kg*cm/s^2, so 1 N = 100 Unreal force units.
	[[nodiscard]] constexpr double NewtonsToUnreal(double Newtons)    { return Newtons * CentimetersPerMeter; }
	[[nodiscard]] constexpr double UnrealToNewtons(double UnrealForce) { return UnrealForce / CentimetersPerMeter; }

	// Imperial -> SI
	[[nodiscard]] constexpr double InchesToMeters(double Inches) { return Inches * MetersPerInch; }
	[[nodiscard]] constexpr double FeetInchesToMeters(double Feet, double Inches) { return (Feet * InchesPerFoot + Inches) * MetersPerInch; }
	[[nodiscard]] constexpr double YardsToMeters(double Yards)   { return Yards * MetersPerYard; }
	[[nodiscard]] constexpr double PoundsToKilograms(double Pounds) { return Pounds * KilogramsPerPound; }
	[[nodiscard]] constexpr double MphToMetersPerSecond(double Mph) { return Mph * MetersPerMile / SecondsPerHour; }

	// SI -> Imperial (display only)
	[[nodiscard]] constexpr double MetersToInches(double Meters) { return Meters / MetersPerInch; }
	[[nodiscard]] constexpr double MetersToYards(double Meters)  { return Meters / MetersPerYard; }
	[[nodiscard]] constexpr double KilogramsToPounds(double Kilograms) { return Kilograms / KilogramsPerPound; }
	[[nodiscard]] constexpr double MetersPerSecondToMph(double MetersPerSecond) { return MetersPerSecond * SecondsPerHour / MetersPerMile; }
}
