#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#include <cmath>
#include <vector>
#include <cmath>
#include <assert.h>
#include <cstdint>
#include <iostream>
using uint16 = uint16_t;
namespace mapping {

template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}
inline int RoundToInt(const float x) { return std::lround(x); }


inline float Odds(float probability) {
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;

//将概率限制在范围[kMinProbability，kMaxProbability]中。
inline float ClampProbability(const float probability) {
  return Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  
  assert(value >= 1);
  assert(value <= 32767);
  return value;
}

extern const std::vector<float>* const kValueToProbability;
// 将uint16（可能有也可能没有更新标记集）转换为[kMinProbability，kMaxProbability]范围内的概率。
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  // namespace mapping

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
