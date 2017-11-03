#include "modules/perception/onboard/common_shared_data.h"

#include "gflags/gflags.h"

namespace apollo {
namespace perception {

DEFINE_int32(
    shared_data_stale_time, 5,
    "the time threshold longer than which the data becomes stale, in second");

DEFINE_int32(stamp_enlarge_factor, 100, "timestamp enlarge factor");

}  // namespace perception
}  // namespace apollo
