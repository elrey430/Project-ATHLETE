// Project ATHLETE

#include "Export/AthleteMjcfExport.h"
#include "Articulation/AthleteCollisionGeometry.h"
#include "Articulation/AthleteJointSetup.h"
#include "Capability/AthleteMotorSkill.h"
#include "Muscle/AthleteMuscleModel.h"

namespace
{
	FString Num(double Value)
	{
		// No "-0" in the file.
		return FString::Printf(TEXT("%.6g"), FMath::Abs(Value) < 1e-12 ? 0.0 : Value);
	}

	FString Vec(const FVector& Value)
	{
		return FString::Printf(TEXT("%s %s %s"), *Num(Value.X), *Num(Value.Y), *Num(Value.Z));
	}

	/** Where a segment's MuJoCo body frame sits (MuJoCo coordinates): at its joint to the parent; the root at its center of mass. */
	FVector BodyOrigin(const FAthleteBodyModel& Model, EAthleteSegment Segment)
	{
		EAthleteJoint Joint;
		return AthleteMjcfExport::ToMuJoCo(AthleteJoints::FindJointToParent(Segment, Joint) ? Model.GetJointCenterM(Joint) : Model.GetSegment(Segment).CenterOfMassM);
	}

	/**
	 * Marker sites on a segment (body frame, Unreal coordinates). The 15 "*_mimic" names are what
	 * LocoMuJoCo's imitation rewards compare against the reference motion; here they sit on anatomical
	 * landmarks (joint centers, mid-hip, head center of mass), like its human skeleton's. Heel and ball
	 * of the foot give the foot's pitch when fitting motion to this body.
	 */
	TArray<TPair<FString, FVector>> GetSites(const FAthleteBodyModel& Model, EAthleteSegment Segment)
	{
		using ES = EAthleteSegment;
		const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
		const FString Side = AthleteSegments::GetSide(Segment) == EAthleteBodySide::Left ? TEXT("left_") : TEXT("right_");
		EAthleteJoint ToParent;
		const FVector JointCenter = AthleteJoints::FindJointToParent(Segment, ToParent) ? Model.GetJointCenterM(ToParent) : Seg.CenterOfMassM;
		switch (AthleteSegments::GetKind(Segment))
		{
		case EAthleteSegmentKind::LowerTrunk:
			return { { TEXT("pelvis_mimic"), 0.5 * (Model.GetJointCenterM(EAthleteJoint::HipLeft) + Model.GetJointCenterM(EAthleteJoint::HipRight)) } };
		case EAthleteSegmentKind::UpperTrunk: return { { TEXT("upper_body_mimic"), JointCenter } }; // xiphion
		case EAthleteSegmentKind::Head:       return { { TEXT("head_mimic"), Seg.CenterOfMassM } };
		case EAthleteSegmentKind::UpperArm:   return { { Side + TEXT("shoulder_mimic"), JointCenter } };
		case EAthleteSegmentKind::Forearm:    return { { Side + TEXT("elbow_mimic"), JointCenter } };
		case EAthleteSegmentKind::Hand:       return { { Side + TEXT("hand_mimic"), JointCenter } }; // wrist
		case EAthleteSegmentKind::Thigh:      return { { Side + TEXT("hip_mimic"), JointCenter } };
		case EAthleteSegmentKind::Shank:      return { { Side + TEXT("knee_mimic"), JointCenter } };
		case EAthleteSegmentKind::Foot:
		{
			// de Leva's foot runs from the heel (origin) to the tip of the longest toe. ESTIMATE: the ball
			// of the foot (metatarsophalangeal joints) at 73% of that length.
			constexpr double BallOfFootFraction = 0.73;
			const FVector Heel(Seg.OriginM.X, Seg.OriginM.Y, 0.0);
			const FVector Toe(Seg.EndM.X, Seg.EndM.Y, 0.0);
			return {
				{ Side + TEXT("foot_mimic"), AthleteCollisionGeometry::ComputeShape(Model, Segment).CenterM },
				{ Side + TEXT("heel"), Heel },
				{ Side + TEXT("ball_of_foot"), Heel + BallOfFootFraction * (Toe - Heel) },
			};
		}
		default: return {};
		}
	}

	struct FWriter
	{
		const FAthleteBodyModel& Model;
		const TArray<FAthleteMjcfHinge>& Hinges;
		FString Out;

		void Line(int32 Depth, const FString& Text)
		{
			Out += FString::ChrN(2 * Depth, TEXT(' ')) + Text + TEXT("\n");
		}

		void Body(EAthleteSegment Segment, const FVector& ParentOrigin, int32 Depth)
		{
			const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
			const FVector Origin = BodyOrigin(Model, Segment);
			Line(Depth, FString::Printf(TEXT("<body name=\"%s\" pos=\"%s\">"), AthleteSegments::GetName(Segment), *Vec(Origin - ParentOrigin)));

			EAthleteJoint ToParent;
			if (AthleteJoints::FindJointToParent(Segment, ToParent))
			{
				for (const FAthleteMjcfHinge& Hinge : Hinges)
				{
					if (Hinge.Joint == ToParent)
					{
						Line(Depth + 1, FString::Printf(TEXT("<joint name=\"%s\" type=\"hinge\" axis=\"%s\" range=\"%s %s\"/>"),
							*Hinge.Name, *Vec(Hinge.Axis), *Num(Hinge.LowerRad), *Num(Hinge.UpperRad)));
					}
				}
			}
			else
			{
				Line(Depth + 1, TEXT("<freejoint name=\"root\"/>"));
			}

			// Principal axes are the body axes in the reference pose, so the inertia stays diagonal (flipping Y changes no moment).
			Line(Depth + 1, FString::Printf(TEXT("<inertial pos=\"%s\" mass=\"%s\" diaginertia=\"%s\"/>"),
				*Vec(AthleteMjcfExport::ToMuJoCo(Seg.CenterOfMassM) - Origin), *Num(Seg.MassKg), *Vec(Seg.PrincipalInertiaKgM2)));

			const FAthleteSegmentCollisionShape Shape = AthleteCollisionGeometry::ComputeShape(Model, Segment);
			const FVector ShapePos = AthleteMjcfExport::ToMuJoCo(Shape.CenterM) - Origin;
			if (Shape.Type == EAthleteCollisionShapeType::Box)
			{
				Line(Depth + 1, FString::Printf(TEXT("<geom name=\"%s\" type=\"box\" pos=\"%s\" size=\"%s\"/>"), AthleteSegments::GetName(Segment), *Vec(ShapePos), *Vec(Shape.BoxHalfExtentsM)));
			}
			else
			{
				// Unreal's capsule half-height includes the rounded end; MuJoCo's is the cylinder's half-length.
				const double CylinderHalfM = FMath::Max(0.0, Shape.CapsuleHalfHeightM - Shape.CapsuleRadiusM);
				Line(Depth + 1, FString::Printf(TEXT("<geom name=\"%s\" type=\"capsule\" pos=\"%s\" size=\"%s %s\"/>"),
					AthleteSegments::GetName(Segment), *Vec(ShapePos), *Num(Shape.CapsuleRadiusM), *Num(CylinderHalfM)));
			}

			for (const TPair<FString, FVector>& Site : GetSites(Model, Segment))
			{
				Line(Depth + 1, FString::Printf(TEXT("<site name=\"%s\" pos=\"%s\" size=\"0.015\" group=\"3\" rgba=\"0.9 0.2 0.2 1\"/>"),
					*Site.Key, *Vec(AthleteMjcfExport::ToMuJoCo(Site.Value) - Origin)));
			}

			for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
			{
				const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
				if (AthleteJoints::GetParentSegment(Joint) == Segment)
				{
					Body(AthleteJoints::GetChildSegment(Joint), Origin, Depth + 1);
				}
			}
			Line(Depth, TEXT("</body>"));
		}
	};
}

TArray<FAthleteMjcfHinge> AthleteMjcfExport::ComputeHinges(const FAthleteBodyModel& Model, const FAthleteMobilityProfile& Mobility, const FAthleteStrengthProfile& Strength)
{
	TArray<FAthleteMjcfHinge> Hinges;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		const FAthleteJointRangeOfMotion Range = Mobility.Get(Joint);
		const FAthleteJointSetup Setup = AthleteJointSetup::Compute(Model, Joint, Range);
		// Gravity only sets the standing stiffness, which the model doesn't carry; the strength limit doesn't depend on it.
		const FAthleteJointImpedance Muscles = AthleteMuscleModel::ComputeStandingImpedance(Model, Joint, Strength, FAthleteMotorSkill(), 9.8);

		// Rebuild the axes from anatomical directions in MuJoCo's right-handed coordinates. Rotating by a
		// positive angle about (Long x Direction) turns the long axis toward Direction.
		const FVector Long = ToMuJoCo(Setup.LongAxis);
		const FVector Flexion = ToMuJoCo(Setup.FlexionDirection);
		const FVector Abduction = ToMuJoCo(Setup.AbductionDirection);
		const FVector AxialReference = ToMuJoCo(Setup.AxialReference);
		const FVector InternalRotation = ToMuJoCo(Setup.InternalRotationDirection);
		const double AxialSign = FMath::Sign((Long ^ AxialReference) | InternalRotation);

		auto Add = [&](const TCHAR* Plane, const FVector& Axis, double NegativeDeg, double PositiveDeg)
		{
			if (NegativeDeg + PositiveDeg <= 0.0)
			{
				return; // locked plane (e.g. the knee doesn't abduct)
			}
			FAthleteMjcfHinge& Hinge = Hinges.AddDefaulted_GetRef();
			Hinge.Joint = Joint;
			Hinge.Plane = Plane;
			Hinge.Name = FString::Printf(TEXT("%s_%s"), AthleteJoints::GetName(Joint), Plane);
			Hinge.Axis = Axis.GetSafeNormal();
			Hinge.LowerRad = -FMath::DegreesToRadians(NegativeDeg);
			Hinge.UpperRad = FMath::DegreesToRadians(PositiveDeg);
			Hinge.TorqueLimitNm = Muscles.TorqueLimitNm;
			Hinge.MaxVelocityRadPerS = Muscles.MaxVelocityRadPerS;
		};
		Add(TEXT("flex"), Long ^ Flexion, Range.ExtensionDeg, Range.FlexionDeg);
		Add(TEXT("abd"), Long ^ Abduction, Range.AdductionDeg, Range.AbductionDeg);
		Add(TEXT("rot"), AxialSign * Long, Range.ExternalRotationDeg, Range.InternalRotationDeg);
	}
	return Hinges;
}

FString AthleteMjcfExport::Export(const FAthleteBodyModel& Model, const FAthleteMobilityProfile& Mobility, const FAthleteStrengthProfile& Strength,
	TConstArrayView<TPair<EAthleteSegment, EAthleteSegment>> IgnoredCollisionPairs, const FAthleteMjcfOptions& Options)
{
	const TArray<FAthleteMjcfHinge> Hinges = ComputeHinges(Model, Mobility, Strength);
	FWriter W{ Model, Hinges };

	W.Line(0, FString::Printf(TEXT("<mujoco model=\"%s\">"), *Options.ModelName));
	W.Line(1, TEXT("<!-- Generated by AthleteMjcfExport from the athlete definition. Don't edit: regenerate. -->"));
	W.Line(1, TEXT("<!-- MuJoCo coordinates: X forward, Y left, Z up (Unreal's body frame with Y flipped). SI units. -->"));
	// Mass properties come from de Leva only: geoms decide contact, never inertia (as in the Chaos body).
	W.Line(1, TEXT("<compiler angle=\"radian\" inertiafromgeom=\"false\" autolimits=\"true\"/>"));
	W.Line(1, FString::Printf(TEXT("<option timestep=\"%s\" gravity=\"0 0 %s\" integrator=\"implicitfast\">"), *Num(Options.TimestepS), *Num(-Options.GravityMps2)));
	W.Line(2, TEXT("<flag energy=\"enable\"/>"));
	W.Line(1, TEXT("</option>"));
	// Stiff contacts and joint limits: time constant (s), critically damped; impedance 0.95-0.99 (see FAthleteMjcfOptions).
	const double TimeConstantS = Options.ConstraintTimeConstantS > 0.0 ? Options.ConstraintTimeConstantS : 2.0 * Options.TimestepS;
	const FString SolRef = FString::Printf(TEXT("%s 1"), *Num(TimeConstantS));
	const TCHAR* SolImp = TEXT("0.95 0.99 0.001");
	W.Line(1, TEXT("<default>"));
	W.Line(2, FString::Printf(TEXT("<joint solreflimit=\"%s\" solimplimit=\"%s\"/>"), *SolRef, SolImp));
	W.Line(2, FString::Printf(TEXT("<geom condim=\"3\" friction=\"%s 0.005 0.0001\" solref=\"%s\" solimp=\"%s\" rgba=\"0.75 0.62 0.5 1\"/>"),
		*Num(Options.FrictionCoefficient), *SolRef, SolImp));
	W.Line(1, TEXT("</default>"));
	W.Line(1, TEXT("<asset>"));
	W.Line(2, TEXT("<texture name=\"grid\" type=\"2d\" builtin=\"checker\" width=\"512\" height=\"512\" rgb1=\"0.22 0.34 0.22\" rgb2=\"0.27 0.4 0.27\"/>"));
	W.Line(2, TEXT("<material name=\"grid\" texture=\"grid\" texrepeat=\"20 20\"/>"));
	W.Line(1, TEXT("</asset>"));

	W.Line(1, TEXT("<worldbody>"));
	W.Line(2, TEXT("<light pos=\"0 0 6\" dir=\"0 0 -1\"/>"));
	if (Options.bIncludeFloor)
	{
		W.Line(2, TEXT("<geom name=\"floor\" type=\"plane\" size=\"50 50 0.1\" material=\"grid\"/>"));
	}
	W.Body(EAthleteSegment::LowerTrunk, FVector::ZeroVector, 2);
	W.Line(1, TEXT("</worldbody>"));

	if (!IgnoredCollisionPairs.IsEmpty())
	{
		W.Line(1, TEXT("<contact>"));
		for (const TPair<EAthleteSegment, EAthleteSegment>& Pair : IgnoredCollisionPairs)
		{
			W.Line(2, FString::Printf(TEXT("<exclude body1=\"%s\" body2=\"%s\"/>"), AthleteSegments::GetName(Pair.Key), AthleteSegments::GetName(Pair.Value)));
		}
		W.Line(1, TEXT("</contact>"));
	}

	// Torque actuators: ctrl is the joint torque in N*m, within isometric strength.
	W.Line(1, TEXT("<actuator>"));
	for (const FAthleteMjcfHinge& Hinge : Hinges)
	{
		W.Line(2, FString::Printf(TEXT("<motor name=\"%s\" joint=\"%s\" ctrlrange=\"%s %s\"/>"), *Hinge.Name, *Hinge.Name, *Num(-Hinge.TorqueLimitNm), *Num(Hinge.TorqueLimitNm)));
	}
	W.Line(1, TEXT("</actuator>"));

	// Data the controller and the validation scripts need, from the same source as the model.
	FString MaxVelocities;
	for (const FAthleteMjcfHinge& Hinge : Hinges)
	{
		MaxVelocities += (MaxVelocities.IsEmpty() ? TEXT("") : TEXT(" ")) + Num(Hinge.MaxVelocityRadPerS);
	}
	W.Line(1, TEXT("<custom>"));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_max_velocity_rad_per_s\" data=\"%s\"/>"), *MaxVelocities));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_hill_curvature\" data=\"%s\"/>"), *Num(AthleteMuscleModel::HillCurvature)));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_eccentric_plateau\" data=\"%s\"/>"), *Num(AthleteMuscleModel::EccentricPlateau)));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_total_mass_kg\" data=\"%s\"/>"), *Num(Model.GetTotalMassKg())));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_center_of_mass_m\" data=\"%s\"/>"), *Vec(ToMuJoCo(Model.GetCenterOfMassM()))));
	W.Line(2, FString::Printf(TEXT("<numeric name=\"athlete_stature_m\" data=\"%s\"/>"), *Num(Model.GetStatureM())));
	W.Line(1, TEXT("</custom>"));
	W.Line(0, TEXT("</mujoco>"));
	return W.Out;
}
