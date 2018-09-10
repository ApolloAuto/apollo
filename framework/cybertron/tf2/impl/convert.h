
#ifndef CYBERTRON_TF2_IMPL_CONVERT_H_
#define CYBERTRON_TF2_IMPL_CONVERT_H_

namespace apollo {
namespace cybertron {
namespace tf2 {
namespace impl {

template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter {
 public:
  template <typename A, typename B>
  static void convert(const A& a, B& b);
};

// The case where both A and B are messages should not happen: if you have two
// messages that are interchangeable, well, that's against the ROS purpose:
// only use one type. Worst comes to worst, specialize the original convert
// function for your types.
// if B == A, the templated version of convert with only one argument will be
// used.
//
// template <>
// template <typename A, typename B>
// inline void Converter<true, true>::convert(const A& a, B& b);

template <>
template <typename A, typename B>
inline void Converter<true, false>::convert(const A& a, B& b) {
  fromMsg(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, true>::convert(const A& a, B& b) {
  b = toMsg(a);
}

template <>
template <typename A, typename B>
inline void Converter<false, false>::convert(const A& a, B& b) {
  fromMsg(toMsg(a), b);
}

}
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_IMPL_CONVERT_H_
