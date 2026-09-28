// Project ATHLETE

#include "Capability/AthleteStrength.h"

namespace
{
	/** One regression row: PeakTorque [Nm] = Intercept + AgeCoeff*age[yr] + HeightCoeff*height[m] + MassCoeff*mass[kg]. */
	struct FHarboRegression
	{
		EAthleteJointAction Action;
		EAthleteStrengthTestMode Mode;
		double TestVelocityDegPerSec;
		double Intercept;
		double AgeCoeff;
		double HeightCoeff;
		double MassCoeff;
	};

	// Harbo T, Brincks J, Andersen H (2012). Maximal isokinetic and isometric muscle strength of
	// major muscle groups related to age, body mass, height, and sex in 178 healthy subjects.
	// Eur J Appl Physiol 112:267-275. Male rows of Table 1 (upper limb) and Table 2 (lower limb).
	// Where the study measured an action both ways, the isometric row is used.
	// Test velocities are from the paper's Methods (knee 90 deg/s; ankle, hip, elbow, shoulder 60 deg/s).
	// The study labels ankle plantar flexion "ankle flexion" and dorsiflexion "ankle extension"
	// (anatomical convention); the magnitudes (~140 Nm vs ~50 Nm for a large man) confirm the mapping.
	using EMode = EAthleteStrengthTestMode;
	const FHarboRegression GHarboMale[] =
	{
		//  Action                                        Mode              deg/s  Intercept    Age   Height   Mass
		{ EAthleteJointAction::KneeExtension,       EMode::Isometric,    0.0, -107.9, -1.40,  158.0, 1.75 },
		{ EAthleteJointAction::KneeFlexion,         EMode::Isokinetic,  90.0,  -90.8, -0.49,   80.0, 0.82 },
		{ EAthleteJointAction::HipExtension,        EMode::Isokinetic,  60.0,  -49.8, -0.97,  103.0, 1.14 },
		{ EAthleteJointAction::HipFlexion,          EMode::Isometric,    0.0,   37.6, -0.72,   36.0, 1.12 },
		{ EAthleteJointAction::AnklePlantarFlexion, EMode::Isokinetic,  60.0,   97.7, -0.72,    4.0, 0.52 },
		{ EAthleteJointAction::AnkleDorsiflexion,   EMode::Isometric,    0.0,  -88.9, -0.04,   68.0, 0.16 },
		{ EAthleteJointAction::ShoulderAbduction,   EMode::Isometric,    0.0,  -30.8, -0.24,   36.0, 0.49 },
		{ EAthleteJointAction::ShoulderAdduction,   EMode::Isokinetic,  60.0,  -72.9, -0.06,   72.0, 0.38 },
		{ EAthleteJointAction::ElbowFlexion,        EMode::Isometric,    0.0,  -23.8, -0.13,   22.0, 0.51 },
		{ EAthleteJointAction::ElbowExtension,      EMode::Isokinetic,  60.0,   -4.4, -0.07,   12.0, 0.42 },
	};
}

FAthleteStrengthProfile FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(double AgeYears, double StatureM, double BodyMassKg)
{
	FAthleteStrengthProfile Profile;
	for (const FHarboRegression& Row : GHarboMale)
	{
		FAthleteJointActionStrength Strength;
		// A regression can go negative far outside the sampled population; torque cannot.
		Strength.PeakTorqueNm = FMath::Max(0.0, Row.Intercept + Row.AgeCoeff * AgeYears + Row.HeightCoeff * StatureM + Row.MassCoeff * BodyMassKg);
		Strength.TestMode = Row.Mode;
		Strength.TestVelocityDegPerSec = Row.TestVelocityDegPerSec;
		Profile.Actions.Add(Row.Action, Strength);
	}
	return Profile;
}

const TCHAR* AthleteStrength::GetActionName(EAthleteJointAction Action)
{
	switch (Action)
	{
	case EAthleteJointAction::HipExtension:        return TEXT("hip_extension");
	case EAthleteJointAction::HipFlexion:          return TEXT("hip_flexion");
	case EAthleteJointAction::KneeExtension:       return TEXT("knee_extension");
	case EAthleteJointAction::KneeFlexion:         return TEXT("knee_flexion");
	case EAthleteJointAction::AnklePlantarFlexion: return TEXT("ankle_plantar_flexion");
	case EAthleteJointAction::AnkleDorsiflexion:   return TEXT("ankle_dorsiflexion");
	case EAthleteJointAction::ShoulderAbduction:   return TEXT("shoulder_abduction");
	case EAthleteJointAction::ShoulderAdduction:   return TEXT("shoulder_adduction");
	case EAthleteJointAction::ElbowFlexion:        return TEXT("elbow_flexion");
	case EAthleteJointAction::ElbowExtension:      return TEXT("elbow_extension");
	default:                                       return TEXT("invalid");
	}
}
