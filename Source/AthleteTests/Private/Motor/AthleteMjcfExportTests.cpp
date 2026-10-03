// Project ATHLETE
// The athlete described for MuJoCo (engine spike after Milestone 4): the exported model must be the
// same body, with the same joints, as the Unreal one. See Export/AthleteMjcfExport.h.

#include "AthleteTestFlags.h"
#include "Articulation/AthleteJointSetup.h"
#include "Definition/AthleteDefinition.h"
#include "Export/AthleteMjcfExport.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "../Physics/AthletePhysicsTestHelpers.h"
#include "XmlFile.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAthleteStrengthProfile BaselineStrength(const FAthleteMorphology& Morphology)
	{
		return FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
	}

	FVector ParseVector(const FString& Text)
	{
		TArray<FString> Parts;
		Text.ParseIntoArray(Parts, TEXT(" "));
		return Parts.Num() == 3 ? FVector(FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2])) : FVector(NAN);
	}

	/** What the MJCF says about the body, read back from the XML (world positions in the reference pose). */
	struct FMjcfSummary
	{
		int32 Bodies = 0;
		int32 Hinges = 0;
		int32 FreeJoints = 0;
		int32 Motors = 0;
		int32 Excludes = 0;
		double MassKg = 0.0;
		FVector MassMoment = FVector::ZeroVector; // sum of mass * world center of mass
		double LowestSoleM = TNumericLimits<double>::Max();
		TArray<FString> Sites;

		void ReadBody(const FXmlNode* Node, const FVector& ParentWorld)
		{
			++Bodies;
			const FVector World = ParentWorld + ParseVector(Node->GetAttribute(TEXT("pos")));
			for (const FXmlNode* Child : Node->GetChildrenNodes())
			{
				const FString& Tag = Child->GetTag();
				if (Tag == TEXT("body"))
				{
					ReadBody(Child, World);
				}
				else if (Tag == TEXT("joint"))
				{
					Hinges += Child->GetAttribute(TEXT("type")) == TEXT("hinge") ? 1 : 0;
				}
				else if (Tag == TEXT("freejoint"))
				{
					++FreeJoints;
				}
				else if (Tag == TEXT("site"))
				{
					Sites.Add(Child->GetAttribute(TEXT("name")));
				}
				else if (Tag == TEXT("inertial"))
				{
					const double Mass = FCString::Atod(*Child->GetAttribute(TEXT("mass")));
					MassKg += Mass;
					MassMoment += Mass * (World + ParseVector(Child->GetAttribute(TEXT("pos"))));
				}
				else if (Tag == TEXT("geom") && Child->GetAttribute(TEXT("name")).StartsWith(TEXT("Foot")))
				{
					const FVector Center = World + ParseVector(Child->GetAttribute(TEXT("pos")));
					LowestSoleM = FMath::Min(LowestSoleM, Center.Z - ParseVector(Child->GetAttribute(TEXT("size"))).Z);
				}
			}
		}

		bool Read(const FString& Xml, FString& OutError)
		{
			const FXmlFile File(Xml, EConstructMethod::ConstructFromBuffer);
			if (!File.IsValid())
			{
				OutError = File.GetLastError();
				return false;
			}
			for (const FXmlNode* Section : File.GetRootNode()->GetChildrenNodes())
			{
				for (const FXmlNode* Node : Section->GetChildrenNodes())
				{
					const FString& Tag = Node->GetTag();
					if (Section->GetTag() == TEXT("worldbody") && Tag == TEXT("body"))
					{
						ReadBody(Node, FVector::ZeroVector);
					}
					Motors += Tag == TEXT("motor") ? 1 : 0;
					Excludes += Tag == TEXT("exclude") ? 1 : 0;
				}
			}
			return true;
		}
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMjcfHingesTest, "Athlete.Motor.Export.HingesMatchAnatomy", AthleteTestFlags)

bool FAthleteMjcfHingesTest::RunTest(const FString& Parameters)
{
	// Every hinge turns the joint the way its name says, in MuJoCo's right-handed coordinates: a small
	// positive rotation moves the limb toward flexion / abduction / internal rotation. Left and right are
	// mirror images, and each hinge carries the joint's strength.
	const FAthleteMorphology Morphology;
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Body model"), FAthleteBodyModel::Build(Morphology, Model));
	const FAthleteMobilityProfile Mobility;
	const TArray<FAthleteMjcfHinge> Hinges = AthleteMjcfExport::ComputeHinges(Model, Mobility, BaselineStrength(Morphology));

	int32 UnlockedPlanes = 0;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const FAthleteJointRangeOfMotion Range = Mobility.Get(AthleteJoints::FromIndex(Index));
		UnlockedPlanes += (Range.FlexionDeg + Range.ExtensionDeg > 0.0) + (Range.AbductionDeg + Range.AdductionDeg > 0.0) + (Range.InternalRotationDeg + Range.ExternalRotationDeg > 0.0);
	}
	UTEST_EQUAL(TEXT("One hinge per unlocked plane"), Hinges.Num(), UnlockedPlanes);

	using namespace AthleteMjcfExport;
	for (const FAthleteMjcfHinge& Hinge : Hinges)
	{
		const FAthleteJointSetup Setup = AthleteJointSetup::Compute(Model, Hinge.Joint, Mobility.Get(Hinge.Joint));
		const FString Plane = Hinge.Plane;
		const FVector Moved = Plane == TEXT("rot") ? ToMuJoCo(Setup.AxialReference) : ToMuJoCo(Setup.LongAxis);
		const FVector Toward = Plane == TEXT("flex") ? ToMuJoCo(Setup.FlexionDirection)
			: Plane == TEXT("abd") ? ToMuJoCo(Setup.AbductionDirection) : ToMuJoCo(Setup.InternalRotationDirection);
		const FVector Rotated = FQuat(Hinge.Axis, FMath::DegreesToRadians(10.0)).RotateVector(Moved);
		TestTrue(FString::Printf(TEXT("%s: +rotation moves toward its anatomical direction"), *Hinge.Name), ((Rotated - Moved) | Toward) > 0.1);
		TestTrue(FString::Printf(TEXT("%s: unit axis"), *Hinge.Name), FMath::IsNearlyEqual(Hinge.Axis.Size(), 1.0, 1e-9));
		TestTrue(FString::Printf(TEXT("%s: range spans zero"), *Hinge.Name), Hinge.LowerRad <= 0.0 && Hinge.UpperRad >= 0.0 && Hinge.UpperRad > Hinge.LowerRad);
		TestTrue(FString::Printf(TEXT("%s: has strength and a speed limit"), *Hinge.Name), Hinge.TorqueLimitNm > 0.0 && Hinge.MaxVelocityRadPerS > 0.0);

		for (const FAthleteMjcfHinge& Other : Hinges)
		{
			if (Other.Joint == Hinge.Joint && &Other != &Hinge)
			{
				TestTrue(FString::Printf(TEXT("%s is perpendicular to %s"), *Hinge.Name, *Other.Name), FMath::Abs(Hinge.Axis | Other.Axis) < 1e-9);
			}
			const EAthleteSegment MirrorChild = AthleteSegments::GetMirror(AthleteJoints::GetChildSegment(Hinge.Joint));
			if (AthleteJoints::GetSide(Hinge.Joint) == EAthleteBodySide::Left && AthleteJoints::GetChildSegment(Other.Joint) == MirrorChild && FCString::Strcmp(Other.Plane, Hinge.Plane) == 0)
			{
				// Mirroring across the midline (Y -> -Y) turns a rotation axis (x, y, z) into (-x, y, -z).
				TestTrue(FString::Printf(TEXT("%s mirrors %s"), *Other.Name, *Hinge.Name), Other.Axis.Equals(FVector(-Hinge.Axis.X, Hinge.Axis.Y, -Hinge.Axis.Z), 1e-9));
				TestTrue(FString::Printf(TEXT("%s has the same range as %s"), *Other.Name, *Hinge.Name), FMath::IsNearlyEqual(Other.LowerRad, Hinge.LowerRad) && FMath::IsNearlyEqual(Other.UpperRad, Hinge.UpperRad));
			}
		}
	}

	// One anchor in absolute terms (MuJoCo: Y is LEFT): hip flexion turns about the axis pointing right.
	const FAthleteMjcfHinge* HipFlexion = Hinges.FindByPredicate([](const FAthleteMjcfHinge& H) { return H.Name == TEXT("HipLeft_flex"); });
	UTEST_NOT_NULL(TEXT("HipLeft_flex exists"), HipFlexion);
	TestTrue(TEXT("Hip flexion axis points right (-Y in MuJoCo)"), HipFlexion->Axis.Equals(FVector(0, -1, 0), 1e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMjcfBodyTest, "Athlete.Motor.Export.MjcfDescribesTheBody", AthleteTestFlags)

bool FAthleteMjcfBodyTest::RunTest(const FString& Parameters)
{
	// The XML, read back, is the Unreal body: 16 segments, the same total mass and center of mass (so
	// every frame offset composes correctly), a motor per hinge, the same ignored contact pairs, and the
	// soles on the floor.
	const FAthleteMorphology Morphology;
	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, false, Morphology));
	const FAthleteBodyModel& Model = Scene.Body->GetBodyModel();
	const FAthleteMobilityProfile Mobility;
	const FAthleteStrengthProfile Strength = BaselineStrength(Morphology);

	const FString Xml = AthleteMjcfExport::Export(Model, Mobility, Strength, Scene.Body->GetIgnoredCollisionPairs());
	FMjcfSummary Summary;
	FString Error;
	UTEST_TRUE(FString::Printf(TEXT("Valid XML (%s)"), *Error), Summary.Read(Xml, Error));

	const int32 ExpectedHinges = AthleteMjcfExport::ComputeHinges(Model, Mobility, Strength).Num();
	TestEqual(TEXT("One body per segment"), Summary.Bodies, AthleteSegments::NumSegments);
	TestEqual(TEXT("One free root joint"), Summary.FreeJoints, 1);
	TestEqual(TEXT("Hinges"), Summary.Hinges, ExpectedHinges);
	TestEqual(TEXT("One motor per hinge"), Summary.Motors, ExpectedHinges);
	TestEqual(TEXT("Same ignored contact pairs"), Summary.Excludes, Scene.Body->GetIgnoredCollisionPairs().Num());
	TestTrue(TEXT("Same total mass (within 1 g)"), FMath::IsNearlyEqual(Summary.MassKg, Model.GetTotalMassKg(), 1e-3));
	const FVector Com = Summary.MassMoment / Summary.MassKg;
	const FVector Expected = AthleteMjcfExport::ToMuJoCo(Model.GetCenterOfMassM());
	TestTrue(FString::Printf(TEXT("Same center of mass (MuJoCo %s vs %s)"), *Com.ToString(), *Expected.ToString()), Com.Equals(Expected, 1e-4));
	TestTrue(FString::Printf(TEXT("Soles on the floor (%.2f mm)"), Summary.LowestSoleM * 1000.0), FMath::Abs(Summary.LowestSoleM) < 1e-4);
	// The marker sites LocoMuJoCo's imitation rewards compare (its humanoids' "sites_for_mimic"), once each.
	for (const TCHAR* Site : { TEXT("upper_body_mimic"), TEXT("head_mimic"), TEXT("pelvis_mimic"),
		TEXT("left_shoulder_mimic"), TEXT("left_elbow_mimic"), TEXT("left_hand_mimic"), TEXT("left_hip_mimic"), TEXT("left_knee_mimic"), TEXT("left_foot_mimic"),
		TEXT("right_shoulder_mimic"), TEXT("right_elbow_mimic"), TEXT("right_hand_mimic"), TEXT("right_hip_mimic"), TEXT("right_knee_mimic"), TEXT("right_foot_mimic") })
	{
		TestEqual(FString::Printf(TEXT("Site %s"), Site), Summary.Sites.FilterByPredicate([Site](const FString& Name) { return Name == Site; }).Num(), 1);
	}
	AddInfo(FString::Printf(TEXT("%d bodies, %d hinges, %d excluded pairs, %.3f kg"), Summary.Bodies, Summary.Hinges, Summary.Excludes, Summary.MassKg));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMjcfExportTool, "ProjectTools.Export.MuJoCo", AthleteTestFlags)

bool FAthleteMjcfExportTool::RunTest(const FString& Parameters)
{
	// Not a test: writes the MuJoCo models to Saved/MuJoCo/ (the reference athlete, the Milestone 2
	// drop-test body, and each athlete asset). Run it after changing an athlete:
	//   powershell -ExecutionPolicy Bypass -File Scripts\RunTests.ps1 -Filter ProjectTools.Export.MuJoCo
	struct FEntry { FString Name; FAthleteMorphology Morphology; FAthleteMobilityProfile Mobility; FAthleteStrengthProfile Strength; };
	TArray<FEntry> Entries;
	Entries.Add({ TEXT("athlete_reference"), FAthleteMorphology(), FAthleteMobilityProfile(), BaselineStrength(FAthleteMorphology()) });
	const FAthleteMorphology Drop = FAthleteMorphology::FromImperial(5, 9, 190);
	Entries.Add({ TEXT("athlete_5ft9_190lb"), Drop, FAthleteMobilityProfile(), BaselineStrength(Drop) });
	for (const TCHAR* Asset : { TEXT("DA_Athlete_Reference"), TEXT("DA_Athlete_A"), TEXT("DA_Athlete_B") })
	{
		const UAthleteDefinition* Definition = LoadObject<UAthleteDefinition>(nullptr, *FString::Printf(TEXT("/Game/Athletes/%s.%s"), Asset, Asset));
		if (!Definition)
		{
			AddWarning(FString::Printf(TEXT("Athlete asset %s not found; skipped"), Asset));
			continue;
		}
		Entries.Add({ FString(Asset).Replace(TEXT("DA_Athlete_"), TEXT("athlete_asset_")).ToLower(), Definition->Morphology, Definition->Mobility, Definition->Strength });
	}

	const FString Directory = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("MuJoCo"));
	IFileManager::Get().MakeDirectory(*Directory, true);
	for (const FEntry& Entry : Entries)
	{
		// Contact pairs to ignore come from the real Unreal body in the same pose.
		FAthletePhysicsTestScene Scene;
		UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, false, Entry.Morphology));
		FAthleteMjcfOptions Options;
		Options.ModelName = Entry.Name;
		const FString Xml = AthleteMjcfExport::Export(Scene.Body->GetBodyModel(), Entry.Mobility, Entry.Strength, Scene.Body->GetIgnoredCollisionPairs(), Options);
		const FString Path = Directory / (Entry.Name + TEXT(".xml"));
		UTEST_TRUE(*FString::Printf(TEXT("Wrote %s"), *Path), FFileHelper::SaveStringToFile(Xml, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
		AddInfo(FString::Printf(TEXT("%s (%.1f kg, %.3f m)"), *Path, Scene.Body->GetBodyModel().GetTotalMassKg(), Scene.Body->GetBodyModel().GetStatureM()));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
