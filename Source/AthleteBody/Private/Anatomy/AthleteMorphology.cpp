// Project ATHLETE

#include "Anatomy/AthleteMorphology.h"
#include "Units/AthleteUnits.h"

namespace
{
	// Hard validity limits: outside these the segment model stops being meaningful.
	// They mirror (and are slightly looser than) the editor ClampMin/ClampMax metadata,
	// because values can also arrive from code, Python, or data files that bypass the editor.
	constexpr double MinStatureM = 1.0;
	constexpr double MaxStatureM = 2.6;
	constexpr double MinBodyMassKg = 30.0;
	constexpr double MaxBodyMassKg = 250.0;
	constexpr double MinScale = 0.5;
	constexpr double MaxScale = 2.0;

	bool IsScaleValid(double Value)
	{
		return FMath::IsFinite(Value) && Value >= MinScale && Value <= MaxScale;
	}
}

double FAthleteMorphology::GetRegionMassScale(EAthleteBodyRegion Region) const
{
	switch (Region)
	{
	case EAthleteBodyRegion::HeadNeck: return HeadNeckMassScale;
	case EAthleteBodyRegion::Trunk:    return TrunkMassScale;
	case EAthleteBodyRegion::Arms:     return ArmMassScale;
	case EAthleteBodyRegion::Legs:     return LegMassScale;
	default:                           checkNoEntry(); return 1.0;
	}
}

bool FAthleteMorphology::Validate(FString* OutError) const
{
	auto Fail = [OutError](const FString& Message)
	{
		if (OutError)
		{
			*OutError = Message;
		}
		return false;
	};

	if (!FMath::IsFinite(StatureM) || StatureM < MinStatureM || StatureM > MaxStatureM)
	{
		return Fail(FString::Printf(TEXT("Stature %.3f m is outside [%.1f, %.1f] m."), StatureM, MinStatureM, MaxStatureM));
	}
	if (!FMath::IsFinite(BodyMassKg) || BodyMassKg < MinBodyMassKg || BodyMassKg > MaxBodyMassKg)
	{
		return Fail(FString::Printf(TEXT("Body mass %.1f kg is outside [%.0f, %.0f] kg."), BodyMassKg, MinBodyMassKg, MaxBodyMassKg));
	}

	const TPair<const TCHAR*, double> Scales[] =
	{
		{ TEXT("LegLengthScale"), LegLengthScale },
		{ TEXT("ArmLengthScale"), ArmLengthScale },
		{ TEXT("FootLengthScale"), FootLengthScale },
		{ TEXT("ShoulderWidthScale"), ShoulderWidthScale },
		{ TEXT("HipWidthScale"), HipWidthScale },
		{ TEXT("HeadNeckMassScale"), HeadNeckMassScale },
		{ TEXT("TrunkMassScale"), TrunkMassScale },
		{ TEXT("ArmMassScale"), ArmMassScale },
		{ TEXT("LegMassScale"), LegMassScale },
	};
	for (const TPair<const TCHAR*, double>& Scale : Scales)
	{
		if (!IsScaleValid(Scale.Value))
		{
			return Fail(FString::Printf(TEXT("%s = %.3f is outside [%.1f, %.1f]."), Scale.Key, Scale.Value, MinScale, MaxScale));
		}
	}
	return true;
}

FAthleteMorphology FAthleteMorphology::FromImperial(double Feet, double Inches, double Pounds, double InAgeYears)
{
	FAthleteMorphology Morphology;
	Morphology.StatureM = AthleteUnits::FeetInchesToMeters(Feet, Inches);
	Morphology.BodyMassKg = AthleteUnits::PoundsToKilograms(Pounds);
	Morphology.AgeYears = InAgeYears;
	return Morphology;
}
