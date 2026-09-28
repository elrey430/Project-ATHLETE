// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteMobility.h"
#include "Anatomy/AthleteMorphology.h"
#include "Capability/AthleteStrength.h"
#include "Engine/DataAsset.h"
#include "AthleteDefinition.generated.h"

class FAthleteBodyModel;

/**
 * An athlete, as data: one asset per athlete in the Content Browser.
 *
 * A Data Asset is a UObject that exists only to hold data. It is created and edited in the
 * editor like any other asset, saved to disk, and referenced by actors or code. Keeping
 * athletes as data (not code) means new athletes need no recompiling.
 *
 * Deliberately NOT one giant "PlayerRating" struct: each category (morphology, strength, and
 * later motor skill, cognition, psychology) is its own struct.
 */
UCLASS(BlueprintType)
class ATHLETEBODY_API UAthleteDefinition : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Identity")
	FText DisplayName;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Body", meta = (ShowOnlyInnerProperties))
	FAthleteMorphology Morphology;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Capability")
	FAthleteStrengthProfile Strength;

	/** Joint ranges of motion (flexibility). Defaults to normal adult (AAOS) values. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Capability")
	FAthleteMobilityProfile Mobility;

	/** Computes the physical body. Returns false (and logs why) if the morphology is invalid. */
	bool BuildBodyModel(FAthleteBodyModel& OutModel, FString* OutError = nullptr) const;

	/**
	 * Replaces Strength with the general-population baseline for this athlete's age, height,
	 * and mass (Harbo et al. 2012). Appears as a button in the asset's Details panel.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Capability")
	void FillStrengthFromGeneralPopulationBaseline();

#if WITH_EDITOR
	/** Shown in the editor's Data Validation results (right-click asset > Validate Assets). */
	virtual EDataValidationResult IsDataValid(class FDataValidationContext& Context) const override;
#endif
};
