// Project ATHLETE

#include "AthleteTrackerBundle.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{
	int64 DtypeSize(const FString& Dtype)
	{
		if (Dtype == TEXT("float64")) return 8;
		if (Dtype == TEXT("float32") || Dtype == TEXT("int32")) return 4;
		if (Dtype == TEXT("uint8")) return 1;
		return 0;
	}

	FCriticalSection CacheLock;
	TMap<FString, TWeakPtr<const FAthleteTrackerBundle>> Cache;
}

int64 FAthleteTrackerArray::Num() const
{
	int64 Count = 1;
	for (const int64 Extent : Shape)
	{
		Count *= Extent;
	}
	return Count;
}

FString FAthleteTrackerBundle::DefaultDirectory()
{
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MuJoCo"), TEXT("unreal")));
}

TSharedPtr<const FAthleteTrackerBundle> FAthleteTrackerBundle::Load(const FString& InDirectory, FString& OutError)
{
	const FString Directory = FPaths::ConvertRelativePathToFull(InDirectory);
	FScopeLock Lock(&CacheLock);
	if (const TWeakPtr<const FAthleteTrackerBundle>* Cached = Cache.Find(Directory))
	{
		if (TSharedPtr<const FAthleteTrackerBundle> Bundle = Cached->Pin())
		{
			return Bundle;
		}
	}

	FString ManifestText;
	if (!FFileHelper::LoadFileToString(ManifestText, *FPaths::Combine(Directory, TEXT("manifest.json"))))
	{
		OutError = FString::Printf(TEXT("No bundle in %s: run Scripts/MuJoCo/export_unreal_bundle.py"), *Directory);
		return nullptr;
	}
	TSharedPtr<FJsonObject> Manifest;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(ManifestText), Manifest) || !Manifest.IsValid())
	{
		OutError = TEXT("manifest.json is not valid JSON");
		return nullptr;
	}

	TSharedRef<FAthleteTrackerBundle> Bundle = MakeShared<FAthleteTrackerBundle>();
	Bundle->Directory = Directory;
	Bundle->ConstantsObject = Manifest->GetObjectField(TEXT("constants"));
	for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : Manifest->GetObjectField(TEXT("arrays"))->Values)
	{
		const TSharedPtr<FJsonObject>& Info = Entry.Value->AsObject();
		FAthleteTrackerArray Array;
		Array.Dtype = Info->GetStringField(TEXT("dtype"));
		for (const TSharedPtr<FJsonValue>& Extent : Info->GetArrayField(TEXT("shape")))
		{
			Array.Shape.Add(static_cast<int64>(Extent->AsNumber()));
		}
		const FString File = FPaths::Combine(Directory, Info->GetStringField(TEXT("file")));
		if (!FFileHelper::LoadFileToArray(Array.Bytes, *File))
		{
			OutError = FString::Printf(TEXT("Cannot read %s"), *File);
			return nullptr;
		}
		const int64 Expected = Array.Num() * DtypeSize(Array.Dtype);
		if (Expected == 0 || Array.Bytes.Num() != Expected)
		{
			OutError = FString::Printf(TEXT("%s: %d bytes, expected %lld (%s)"), *File, Array.Bytes.Num(), Expected, *Array.Dtype);
			return nullptr;
		}
		Bundle->Arrays.Add(Entry.Key, MoveTemp(Array));
	}

	Cache.Add(Directory, Bundle);
	return Bundle;
}

const FAthleteTrackerArray* FAthleteTrackerBundle::FindTyped(const FString& Name, const TCHAR* Dtype) const
{
	const FAthleteTrackerArray* Array = Arrays.Find(Name);
	return Array && Array->Dtype == Dtype ? Array : nullptr;
}

mjModel* FAthleteTrackerBundle::LoadModel(FString& OutError) const
{
	const FString Path = FPaths::Combine(Directory, TEXT("model.mjb"));
	mjModel* Model = mj_loadModel(TCHAR_TO_UTF8(*Path), nullptr);
	if (!Model)
	{
		OutError = FString::Printf(TEXT("MuJoCo could not load %s"), *Path);
	}
	return Model;
}

TArray<int32> AthleteTrackerJson::IntArray(const FJsonObject& Object, const FString& Field)
{
	TArray<int32> Values;
	for (const TSharedPtr<FJsonValue>& Value : Object.GetArrayField(Field))
	{
		Values.Add(static_cast<int32>(Value->AsNumber()));
	}
	return Values;
}

TArray<double> AthleteTrackerJson::NumberArray(const FJsonObject& Object, const FString& Field)
{
	TArray<double> Values;
	for (const TSharedPtr<FJsonValue>& Value : Object.GetArrayField(Field))
	{
		Values.Add(Value->AsNumber());
	}
	return Values;
}
