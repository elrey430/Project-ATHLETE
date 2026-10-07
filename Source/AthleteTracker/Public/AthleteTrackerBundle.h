// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

/** One array of the bundle: raw little-endian values (manifest.json gives dtype and shape). */
struct ATHLETETRACKER_API FAthleteTrackerArray
{
	FString Dtype;
	TArray<int64> Shape;
	TArray<uint8> Bytes;

	int64 Num() const;
	int64 Dim(int32 Axis) const { return Shape.IsValidIndex(Axis) ? Shape[Axis] : 1; }
	template <typename T> const T* Data() const { return reinterpret_cast<const T*>(Bytes.GetData()); }
};

/**
 * Everything the learned athlete needs, exported from Python (Scripts/MuJoCo/export_unreal_bundle.py):
 * the compiled MuJoCo model, the policy weights, the motion-matching database, constants, and a golden
 * session for parity tests. Loaded once per folder and shared (the database is a few hundred MB).
 */
class ATHLETETRACKER_API FAthleteTrackerBundle
{
public:
	/** The default folder: Saved/MuJoCo/unreal. */
	static FString DefaultDirectory();

	/** Loads (or returns the already loaded) bundle in Directory. Null on failure, with OutError set. */
	static TSharedPtr<const FAthleteTrackerBundle> Load(const FString& Directory, FString& OutError);

	const FAthleteTrackerArray* Find(const FString& Name) const { return Arrays.Find(Name); }

	/** The array if it exists with this dtype ("float32", "float64", "int32", "uint8"), else null. */
	const FAthleteTrackerArray* FindTyped(const FString& Name, const TCHAR* Dtype) const;

	/** A fresh copy of the MuJoCo model (caller frees it with mj_deleteModel). */
	mjModel* LoadModel(FString& OutError) const;

	const FJsonObject& Constants() const { return *ConstantsObject; }

	FString Directory;

private:
	TMap<FString, FAthleteTrackerArray> Arrays;
	TSharedPtr<FJsonObject> ConstantsObject;
};

/** Small helpers for reading constants. */
namespace AthleteTrackerJson
{
	ATHLETETRACKER_API TArray<int32> IntArray(const FJsonObject& Object, const FString& Field);
	ATHLETETRACKER_API TArray<double> NumberArray(const FJsonObject& Object, const FString& Field);
}
