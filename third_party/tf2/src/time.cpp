#include "tf2/time.h"

namespace tf2 {
  double time_to_sec(Time t) { return static_cast<double>(t) / 1e9; }
}
