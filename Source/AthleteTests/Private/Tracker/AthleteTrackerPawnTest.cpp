// Project ATHLETE

#include "AthleteTestFlags.h"
#include "AthleteTrackerPawn.h"
#include "AthleteTrackerSim.h"
#include "Engine/World.h"
#include "Tests/AutomationCommon.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS

/**
 * The playable learned athlete in an Unreal world: spawned like the game mode does, ticked at 60 fps.
 * It must draw every body shape where MuJoCo has it, stand for 2 s, then walk when told to.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerPawnTest, "Athlete.Tracker.Pawn.StandsAndWalksInAWorld", AthleteTestFlags)

bool FAthleteTrackerPawnTest::RunTest(const FString& Parameters)
{
	constexpr float Frame = 1.0f / 60.0f;
	FTestWorldWrapper TestWorld;
	UTEST_TRUE(TEXT("Create test world"), TestWorld.CreateTestWorld(EWorldType::Game));
	UWorld* World = TestWorld.GetTestWorld();
	AAthleteTrackerPawn* Pawn = World->SpawnActor<AAthleteTrackerPawn>(FVector(500.0, 300.0, 0.0), FRotator::ZeroRotator);
	UTEST_NOT_NULL(TEXT("Spawn the pawn"), Pawn);
	UTEST_TRUE(TEXT("Begin play"), TestWorld.BeginPlayInTestWorld());
	const FAthleteTrackerSim* Sim = Pawn->Simulation();
	UTEST_NOT_NULL(TEXT("The bundle loaded and the simulation started"), Sim);

	// 16 visible MuJoCo geoms: 5 boxes (trunk x3, feet) and 11 capsules (a cylinder and two spheres each).
	TestEqual(TEXT("Every body shape is drawn"), Pawn->NumShapeInstances(), 5 + 3 * 11);

	for (int32 I = 0; I < 120; ++I)
	{
		TestWorld.TickTestWorld(Frame);
	}
	const FVector StandM = Sim->PelvisPositionM();
	TestFalse(TEXT("Stands for 2 s"), Sim->HasFallen());

	Pawn->SetCommand({1.5, 0.0, 0.0});
	for (int32 I = 0; I < 240; ++I)
	{
		TestWorld.TickTestWorld(Frame);
	}
	TestWorld.ForwardErrorMessages(this);
	const FVector WalkM = Sim->PelvisPositionM();
	const double Distance = FVector::Dist2D(StandM, WalkM);
	AddInfo(FString::Printf(TEXT("Walked %.2f m in 4 s (command 1.5 m/s, shaped from standing); %.0f steps simulated"), Distance, static_cast<double>(Sim->StepCount())));
	TestFalse(TEXT("Walks without falling"), Sim->HasFallen());
	TestTrue(TEXT("Walks at least 3.5 m in 4 s"), Distance > 3.5);
	TestTrue(TEXT("Real time: one 10 ms step per 10 ms of game time (+-3 steps)"), FMath::Abs(Sim->StepCount() - 600) <= 3);

	// The pelvis shape is drawn where MuJoCo has the pelvis (geom 1, LowerTrunk), in Unreal units.
	const mjData* D = Sim->Data();
	const FVector Expected = Pawn->ToWorld(FVector(D->geom_xpos[3], D->geom_xpos[4], D->geom_xpos[5]));
	FTransform Drawn;
	UTEST_TRUE(TEXT("The pelvis box is drawn"), Pawn->GetDrawnShape(1, Drawn));
	AddInfo(FString::Printf(TEXT("Pelvis drawn %.2f cm from MuJoCo's pelvis geom; box scale %s"),
		FVector::Dist(Expected, Drawn.GetLocation()), *Drawn.GetScale3D().ToString()));
	TestTrue(TEXT("Pelvis is drawn within 1 cm of MuJoCo's pelvis"), FVector::Dist(Expected, Drawn.GetLocation()) < 1.0);
	// Its up axis matches MuJoCo's (Y mirrored): the box's local Z in the world.
	const FVector UpMuJoCo(D->geom_xmat[9 * 1 + 2], -D->geom_xmat[9 * 1 + 5], D->geom_xmat[9 * 1 + 8]);
	TestTrue(TEXT("Pelvis orientation matches"), FVector::DotProduct(Drawn.GetRotation().GetAxisZ(), UpMuJoCo) > 0.999);
	return true;
}

#endif
