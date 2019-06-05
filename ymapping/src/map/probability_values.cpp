#include "ymapping/map/probability_values.h"
#include <iostream>
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
float SlowValueToProbability(const uint16 value) {
  if (value == kUnknownProbabilityValue) {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);           //应该要减kScale
}

/*
查表法,快速.类似于9*9乘法表的作用,vector存储的是提前计算好的value到p的映射。
提前计算好[1,32767]到[0.1,0.9]之间的映射关系。
返回值：vector<float>: 长度size()=32767*2.
即[0,1,2,...,32767,
   0,1,2,...,32767]
*/
//2份value:前一半对应没有hit，后一半对应hit。
const std::vector<float>* PrecomputeValueToProbability() {   
  std::cout<<"test"<<std::endl;
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  //重复两次，以便 有 和 没有 更新标记的值都可以转换为概率。
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;    // size()=32767*2
}

}  // namespace

//定义,计算value到p的映射：[0,32767]->[0.1,0.9]
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

/*
参数：odds，
返回值：vector<uint16>,size()=1+32767.
之前没有hit过，则没有update，按论文公式(2)计算：
求p,
求[1,32767],
求[1,32767]+32768
push_back()
之前有hit过，则有update，按论文公式(3)计算：
求(*kValueToProbability)[cell]->[0.1,0.9]即原始p
求p'=odds*Odds(p)
求p'映射到[1,32767]
push_back():[1,32767]+32768.
*/
// table[0]:对应未更新过的  table[1-32767]:对应更新过
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);    //标记没有update
  // 当probability>0.5 会增大value   当probability<0.5 会减小value
  for (int cell = 1; cell != 32768; ++cell) {     //标记有update
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
