// Project ATHLETE

#include "Anatomy/AthleteSegments.h"

EAthleteSegmentKind AthleteSegments::GetKind(EAthleteSegment Segment)
{
	switch (Segment)
	{
	case EAthleteSegment::LowerTrunk:    return EAthleteSegmentKind::LowerTrunk;
	case EAthleteSegment::MiddleTrunk:   return EAthleteSegmentKind::MiddleTrunk;
	case EAthleteSegment::UpperTrunk:    return EAthleteSegmentKind::UpperTrunk;
	case EAthleteSegment::Head:          return EAthleteSegmentKind::Head;
	case EAthleteSegment::UpperArmLeft:
	case EAthleteSegment::UpperArmRight: return EAthleteSegmentKind::UpperArm;
	case EAthleteSegment::ForearmLeft:
	case EAthleteSegment::ForearmRight:  return EAthleteSegmentKind::Forearm;
	case EAthleteSegment::HandLeft:
	case EAthleteSegment::HandRight:     return EAthleteSegmentKind::Hand;
	case EAthleteSegment::ThighLeft:
	case EAthleteSegment::ThighRight:    return EAthleteSegmentKind::Thigh;
	case EAthleteSegment::ShankLeft:
	case EAthleteSegment::ShankRight:    return EAthleteSegmentKind::Shank;
	case EAthleteSegment::FootLeft:
	case EAthleteSegment::FootRight:     return EAthleteSegmentKind::Foot;
	default:                             checkNoEntry(); return EAthleteSegmentKind::Head;
	}
}

EAthleteBodyRegion AthleteSegments::GetRegion(EAthleteSegment Segment)
{
	switch (GetKind(Segment))
	{
	case EAthleteSegmentKind::Head:        return EAthleteBodyRegion::HeadNeck;
	case EAthleteSegmentKind::UpperTrunk:
	case EAthleteSegmentKind::MiddleTrunk:
	case EAthleteSegmentKind::LowerTrunk:  return EAthleteBodyRegion::Trunk;
	case EAthleteSegmentKind::UpperArm:
	case EAthleteSegmentKind::Forearm:
	case EAthleteSegmentKind::Hand:        return EAthleteBodyRegion::Arms;
	default:                               return EAthleteBodyRegion::Legs;
	}
}

EAthleteBodySide AthleteSegments::GetSide(EAthleteSegment Segment)
{
	switch (Segment)
	{
	case EAthleteSegment::UpperArmLeft:
	case EAthleteSegment::ForearmLeft:
	case EAthleteSegment::HandLeft:
	case EAthleteSegment::ThighLeft:
	case EAthleteSegment::ShankLeft:
	case EAthleteSegment::FootLeft:      return EAthleteBodySide::Left;
	case EAthleteSegment::UpperArmRight:
	case EAthleteSegment::ForearmRight:
	case EAthleteSegment::HandRight:
	case EAthleteSegment::ThighRight:
	case EAthleteSegment::ShankRight:
	case EAthleteSegment::FootRight:     return EAthleteBodySide::Right;
	default:                             return EAthleteBodySide::Center;
	}
}

EAthleteSegment AthleteSegments::GetMirror(EAthleteSegment Segment)
{
	switch (Segment)
	{
	case EAthleteSegment::UpperArmLeft:  return EAthleteSegment::UpperArmRight;
	case EAthleteSegment::ForearmLeft:   return EAthleteSegment::ForearmRight;
	case EAthleteSegment::HandLeft:      return EAthleteSegment::HandRight;
	case EAthleteSegment::ThighLeft:     return EAthleteSegment::ThighRight;
	case EAthleteSegment::ShankLeft:     return EAthleteSegment::ShankRight;
	case EAthleteSegment::FootLeft:      return EAthleteSegment::FootRight;
	case EAthleteSegment::UpperArmRight: return EAthleteSegment::UpperArmLeft;
	case EAthleteSegment::ForearmRight:  return EAthleteSegment::ForearmLeft;
	case EAthleteSegment::HandRight:     return EAthleteSegment::HandLeft;
	case EAthleteSegment::ThighRight:    return EAthleteSegment::ThighLeft;
	case EAthleteSegment::ShankRight:    return EAthleteSegment::ShankLeft;
	case EAthleteSegment::FootRight:     return EAthleteSegment::FootLeft;
	default:                             return Segment;
	}
}

const TCHAR* AthleteSegments::GetName(EAthleteSegment Segment)
{
	switch (Segment)
	{
	case EAthleteSegment::LowerTrunk:    return TEXT("LowerTrunk");
	case EAthleteSegment::MiddleTrunk:   return TEXT("MiddleTrunk");
	case EAthleteSegment::UpperTrunk:    return TEXT("UpperTrunk");
	case EAthleteSegment::Head:          return TEXT("Head");
	case EAthleteSegment::UpperArmLeft:  return TEXT("UpperArmLeft");
	case EAthleteSegment::ForearmLeft:   return TEXT("ForearmLeft");
	case EAthleteSegment::HandLeft:      return TEXT("HandLeft");
	case EAthleteSegment::UpperArmRight: return TEXT("UpperArmRight");
	case EAthleteSegment::ForearmRight:  return TEXT("ForearmRight");
	case EAthleteSegment::HandRight:     return TEXT("HandRight");
	case EAthleteSegment::ThighLeft:     return TEXT("ThighLeft");
	case EAthleteSegment::ShankLeft:     return TEXT("ShankLeft");
	case EAthleteSegment::FootLeft:      return TEXT("FootLeft");
	case EAthleteSegment::ThighRight:    return TEXT("ThighRight");
	case EAthleteSegment::ShankRight:    return TEXT("ShankRight");
	case EAthleteSegment::FootRight:     return TEXT("FootRight");
	default:                             return TEXT("Invalid");
	}
}
