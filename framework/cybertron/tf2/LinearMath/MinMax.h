
#ifndef CYBERTRON_TF2_LINEARMATH_MINMAX_H_
#define CYBERTRON_TF2_LINEARMATH_MINMAX_H_

template <typename T>
TF2SIMD_FORCE_INLINE const T& tf2Min(const T& a, const T& b) {
  return a < b ? a : b;
}

template <typename T>
TF2SIMD_FORCE_INLINE const T& tf2Max(const T& a, const T& b) {
  return a > b ? a : b;
}

template <typename T>
TF2SIMD_FORCE_INLINE const T& GEN_clamped(const T& a, const T& lb,
                                          const T& ub) {
  return a < lb ? lb : (ub < a ? ub : a);
}

template <typename T>
TF2SIMD_FORCE_INLINE void tf2SetMin(T& a, const T& b) {
  if (b < a) {
    a = b;
  }
}

template <typename T>
TF2SIMD_FORCE_INLINE void tf2SetMax(T& a, const T& b) {
  if (a < b) {
    a = b;
  }
}

template <typename T>
TF2SIMD_FORCE_INLINE void GEN_clamp(T& a, const T& lb, const T& ub) {
  if (a < lb) {
    a = lb;
  } else if (ub < a) {
    a = ub;
  }
}

#endif
