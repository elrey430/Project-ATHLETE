// Project ATHLETE

#include "Anatomy/AthleteMobility.h"

namespace
{
	FAthleteJointRangeOfMotion MakeRom(double Flexion, double Extension, double Abduction, double Adduction, double Internal, double External)
	{
		FAthleteJointRangeOfMotion Rom;
		Rom.FlexionDeg = Flexion;
		Rom.ExtensionDeg = Extension;
		Rom.AbductionDeg = Abduction;
		Rom.AdductionDeg = Adduction;
		Rom.InternalRotationDeg = Internal;
		Rom.ExternalRotationDeg = External;
		return Rom;
	}
}

FAthleteMobilityProfile::FAthleteMobilityProfile()
{
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		Joints.Add(Joint, GetNormalAdult(AthleteJoints::GetKind(Joint)));
	}
}

FAthleteJointRangeOfMotion FAthleteMobilityProfile::Get(EAthleteJoint Joint) const
{
	if (const FAthleteJointRangeOfMotion* Rom = Joints.Find(Joint))
	{
		return *Rom;
	}
	return GetNormalAdult(AthleteJoints::GetKind(Joint));
}

FAthleteJointRangeOfMotion FAthleteMobilityProfile::GetNormalAdult(EAthleteJointKind Kind)
{
	switch (Kind)
	{
	//                                  Flex   Ext   Abd   Add   Int   Ext
	// Thoracolumbar totals (80/25/35/45) split between two joints. ESTIMATE split: the lumbar
	// spine contributes most flexion; the thoracic spine most rotation.
	case EAthleteJointKind::Lumbar:   return MakeRom( 55.0, 15.0, 20.0, 20.0, 10.0, 10.0);
	case EAthleteJointKind::Thoracic: return MakeRom( 25.0, 10.0, 15.0, 15.0, 35.0, 35.0);
	// AAOS cervical: flexion 45, extension 45, lateral flexion 45, rotation 60.
	case EAthleteJointKind::Neck:     return MakeRom( 45.0, 45.0, 45.0, 45.0, 60.0, 60.0);
	// AAOS: flexion 180, extension 60, abduction 180, internal 70, external 90. Adduction ESTIMATE.
	case EAthleteJointKind::Shoulder: return MakeRom(180.0, 60.0,180.0, 45.0, 70.0, 90.0);
	// AAOS elbow flexion 150, extension 0; forearm pronation 80 / supination 80 carried on this joint.
	case EAthleteJointKind::Elbow:    return MakeRom(150.0,  0.0,  0.0,  0.0, 80.0, 80.0);
	// AAOS wrist: flexion 80, extension 70, radial deviation 20, ulnar deviation 30.
	case EAthleteJointKind::Wrist:    return MakeRom( 80.0, 70.0, 20.0, 30.0,  0.0,  0.0);
	// AAOS hip: flexion 120, extension 30, abduction 45, adduction 30, internal 45, external 45.
	case EAthleteJointKind::Hip:      return MakeRom(120.0, 30.0, 45.0, 30.0, 45.0, 45.0);
	// AAOS knee flexion 135. Hinge for now: tibial rotation of the flexed knee is not modeled.
	case EAthleteJointKind::Knee:     return MakeRom(135.0,  0.0,  0.0,  0.0,  0.0,  0.0);
	// AAOS ankle: dorsiflexion 20, plantarflexion 50, inversion 35, eversion 15. Toe-in/out ESTIMATE.
	case EAthleteJointKind::Ankle:    return MakeRom( 20.0, 50.0, 10.0, 10.0, 35.0, 15.0);
	default:                          checkNoEntry(); return FAthleteJointRangeOfMotion();
	}
}
