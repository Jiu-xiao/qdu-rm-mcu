#pragma once

#include "comp_utils.hpp"

namespace Component {
class Triangle {
 public:
  typedef struct {
    float angle[3] = {0.0f, 0.0f, 0.0f}; /* 内角 */
    float side[3] = {0.0f, 0.0f, 0.0f};  /* 对边 */
  } Data;

  bool Slove();

  static float Supplementary(float angle);

  static float Reciprocal(float angle);

  static float InvSinThrm(float A, float a, float b);

  static float InvCosThrm(float a, float b, float c);

  static float SinThrm(float A, float a, float B);

  static float CosThrm(float a, float b, float C);

  void Reset();

  Data data_;
};
}  // namespace Component
