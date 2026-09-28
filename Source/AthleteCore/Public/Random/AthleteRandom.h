// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

/**
 * Deterministic, seedable random number stream (PCG32, M.E. O'Neill, pcg-random.org).
 *
 * Why not FMath::Rand or FRandomStream?
 *  - FMath::Rand is global state: any system calling it shifts every other system's sequence.
 *  - FRandomStream's algorithm is owned by Epic and could change between engine versions,
 *    silently breaking the reproduction of a recorded seed.
 * PCG32 is small, fast, statistically strong, and its output for a given seed is fixed forever.
 * The automation tests pin it to the published reference output.
 *
 * Independent streams: each consumer (e.g. "Perception", "Athlete7.Motor") should own its own
 * stream derived from the master seed with MakeSubstream(). Adding a new consumer then never
 * changes the numbers any existing consumer receives, which keeps old recordings reproducible.
 *
 * Randomness is NOT a substitute for causes. ATHLETE uses it only for genuine natural variance
 * (e.g. neuromuscular noise), never to decide outcomes like "did the tackle succeed".
 *
 * Not thread-safe: each stream must be used by one thread at a time.
 */
class FAthleteRandomStream
{
public:
	/** Default stream constant from the PCG reference implementation. */
	static constexpr uint64 DefaultSequence = 0xda3e39cb94b95bdbULL;

	FAthleteRandomStream()
	{
		Seed(0, DefaultSequence);
	}

	explicit FAthleteRandomStream(uint64 InSeed, uint64 InSequence = DefaultSequence)
	{
		Seed(InSeed, InSequence);
	}

	/** Re-seeds the stream. Identical (Seed, Sequence) pairs always produce identical output. */
	void Seed(uint64 InSeed, uint64 InSequence = DefaultSequence)
	{
		InitialSeed = InSeed;
		InitialSequence = InSequence;

		// Seeding procedure matches pcg32_srandom_r() in the reference implementation.
		State = 0;
		Increment = (InSequence << 1u) | 1u; // Increment must be odd.
		NextUInt32();
		State += InSeed;
		NextUInt32();
	}

	/** Uniformly distributed 32-bit value. */
	uint32 NextUInt32()
	{
		const uint64 OldState = State;
		State = OldState * Multiplier + Increment;
		const uint32 XorShifted = static_cast<uint32>(((OldState >> 18u) ^ OldState) >> 27u);
		const uint32 Rotation = static_cast<uint32>(OldState >> 59u);
		return (XorShifted >> Rotation) | (XorShifted << ((0u - Rotation) & 31u));
	}

	/** Uniform double in [0, 1). Uses 53 random bits so every representable step is reachable. */
	double NextUnitDouble()
	{
		const uint64 High = static_cast<uint64>(NextUInt32()) >> 5; // 27 bits
		const uint64 Low  = static_cast<uint64>(NextUInt32()) >> 6; // 26 bits
		return static_cast<double>((High << 26) | Low) * (1.0 / 9007199254740992.0); // 2^53
	}

	/** Uniform double in [Min, Max). */
	double NextDoubleInRange(double Min, double Max)
	{
		return Min + (Max - Min) * NextUnitDouble();
	}

	/**
	 * Derives an independent child stream for a named consumer.
	 * The child depends only on this stream's seed/sequence identity and the name,
	 * NOT on how many numbers this stream has already produced.
	 */
	FAthleteRandomStream MakeSubstream(FStringView ConsumerName) const
	{
		const uint64 NameHash = HashName(ConsumerName);
		return FAthleteRandomStream(Mix64(InitialSeed ^ NameHash), Mix64(InitialSequence + NameHash));
	}

	uint64 GetInitialSeed() const { return InitialSeed; }

private:
	/** FNV-1a over UTF-16 code units: stable across platforms and engine versions (unlike GetTypeHash). */
	static uint64 HashName(FStringView Name)
	{
		uint64 Hash = 0xcbf29ce484222325ULL;
		for (const TCHAR Character : Name)
		{
			Hash ^= static_cast<uint64>(static_cast<uint16>(Character));
			Hash *= 0x100000001b3ULL;
		}
		return Hash;
	}

	/** SplitMix64 finalizer: spreads similar inputs to unrelated outputs. */
	static uint64 Mix64(uint64 Value)
	{
		Value += 0x9e3779b97f4a7c15ULL;
		Value = (Value ^ (Value >> 30)) * 0xbf58476d1ce4e5b9ULL;
		Value = (Value ^ (Value >> 27)) * 0x94d049bb133111ebULL;
		return Value ^ (Value >> 31);
	}

	static constexpr uint64 Multiplier = 6364136223846793005ULL;

	uint64 State = 0;
	uint64 Increment = 1;

	// Remembered so substreams can be derived without depending on consumption history.
	uint64 InitialSeed = 0;
	uint64 InitialSequence = DefaultSequence;
};
