// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Components/LineBatchComponent.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/World.h"
#include "Lab/AthleteLabBodyPreview.h"
#include "Tests/AutomationCommon.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * The body preview must actually submit debug geometry when it ticks. Rendering can't be
 * checked headlessly, but the world's debug line batcher (where every DrawDebug* call lands)
 * can: after one tick it must hold the preview's capsule and sphere lines.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteLabBodyPreviewDrawTest, "Athlete.Lab.BodyPreview.DrawsWhenTicked", AthleteTestFlags)

bool FAthleteLabBodyPreviewDrawTest::RunTest(const FString& Parameters)
{
	constexpr float FrameDeltaSeconds = 1.0f / 60.0f;

	FTestWorldWrapper TestWorld;
	UTEST_TRUE(TEXT("Create test world"), TestWorld.CreateTestWorld(EWorldType::Game));
	UWorld* World = TestWorld.GetTestWorld();

	UAthleteDefinition* Athlete = NewObject<UAthleteDefinition>(GetTransientPackage());
	AAthleteLabBodyPreview* Preview = World->SpawnActor<AAthleteLabBodyPreview>();
	UTEST_NOT_NULL(TEXT("Spawn preview"), Preview);
	Preview->SetAthlete(Athlete);

	UTEST_TRUE(TEXT("Begin play"), TestWorld.BeginPlayInTestWorld());
	TestWorld.TickTestWorld(FrameDeltaSeconds);
	TestWorld.ForwardErrorMessages(this);

	TestTrue(TEXT("Preview built a valid body model"), Preview->HasValidModel());

	const ULineBatchComponent* LineBatcher = World->GetLineBatcher(UWorld::ELineBatcherType::World);
	UTEST_NOT_NULL(TEXT("World has a debug line batcher"), LineBatcher);
	const int32 LineCount = LineBatcher->BatchedLines.Num();
	TestTrue(*FString::Printf(TEXT("Preview drew debug lines (got %d)"), LineCount), LineCount > 0);

	TestWorld.EndPlayInTestWorld();
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
