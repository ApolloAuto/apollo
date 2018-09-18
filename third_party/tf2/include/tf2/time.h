#ifndef TF2_TIME_H
#define TF2_TIME_H

#include <stdint.h>
#include <limits>

namespace tf2 {
typedef uint64_t Time;
typedef uint64_t Duration;
const uint64_t TIME_MAX = std::numeric_limits<uint64_t>::max();
inline double time_to_sec(Time t) { return static_cast<double>(t) / 1e9; }
}
#endif
