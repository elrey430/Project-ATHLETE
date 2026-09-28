// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteBodyModel.h"
#include "GameFramework/Actor.h"
#include "AthleteLabBodyPreview.generated.h"

class UAthleteDefinition;

/**
 * Lab instrument: draws an athlete's computed body in the reference pose.
 *
 *  - Capsules: one per segment. Radius is the radius of a cylinder with the segment's length and
 *    mass at body density, so thicker capsules really do mean heavier segments. VISUAL ONLY;
 *    Milestone 2 defines real collision geometry.
 *  - Small yellow spheres: segment centers of mass.
 *  - Magenta sphere: whole-body center of mass, with a line down to the floor. That floor point
 *    is what balance will be about in Milestone 3.
 *  - Left side blue, right side red, midline white.
 *
 * Works in the editor viewport without pressing Play and redraws live as you edit the athlete
 * asset. On BeginPlay it records the body to telemetry (Saved/Telemetry).
 * Console: athlete.Debug.Anatomy 0/1 hides/shows all previews.
 *
 * No physics, no simulation: this only displays what FAthleteBodyModel computed.
 */
UCLASS()
class PROJECTATHLETE_API AAthleteLabBodyPreview : public AActor
{
	GENERATED_BODY()

public:
	AAthleteLabBodyPreview();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Lets the actor tick (and therefore draw) in editor viewports, not only during Play. */
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }

	const FAthleteBodyModel& GetBodyModel() const { return Model; }
	bool HasValidModel() const { return bModelValid; }

	void SetAthlete(UAthleteDefinition* InAthlete) { Athlete = InAthlete; }

protected:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Preview")
	TObjectPtr<UAthleteDefinition> Athlete;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bDrawSegments = true;

	UPROPERTY(EditAnywhere, Category = "Preview")
	bool bDrawCentersOfMass = true;

private:
	/** Rebuilds the body model from the athlete asset. Cheap (16 segments), so done every tick. */
	void RebuildModel();
	void Draw() const;
	void RecordTelemetry() const;

	FAthleteBodyModel Model;
	bool bModelValid = false;
};
