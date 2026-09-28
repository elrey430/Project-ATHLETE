// Project ATHLETE

#include "Definition/AthleteDefinition.h"
#include "AthleteBody.h"
#include "Anatomy/AthleteBodyModel.h"

#if WITH_EDITOR
#include "Misc/DataValidation.h"
#endif

bool UAthleteDefinition::BuildBodyModel(FAthleteBodyModel& OutModel, FString* OutError) const
{
	FString Error;
	if (!FAthleteBodyModel::Build(Morphology, OutModel, &Error))
	{
		UE_LOG(LogAthleteBody, Warning, TEXT("Athlete '%s' has an invalid body: %s"), *GetName(), *Error);
		if (OutError)
		{
			*OutError = Error;
		}
		return false;
	}
	return true;
}

void UAthleteDefinition::FillStrengthFromGeneralPopulationBaseline()
{
	// Modify() records the change for Undo and marks the asset as needing to be saved.
	Modify();
	Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
}

#if WITH_EDITOR
EDataValidationResult UAthleteDefinition::IsDataValid(FDataValidationContext& Context) const
{
	EDataValidationResult Result = Super::IsDataValid(Context);

	FAthleteBodyModel Model;
	FString Error;
	if (!FAthleteBodyModel::Build(Morphology, Model, &Error))
	{
		Context.AddError(FText::FromString(Error));
		Result = EDataValidationResult::Invalid;
	}
	return Result;
}
#endif
