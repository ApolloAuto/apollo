#ifndef apollo_PERCEPTION_LIB_BASE_TIME_UTIL_H
#define apollo_PERCEPTION_LIB_BASE_TIME_UTIL_H

#include <sys/time.h>
#include <iomanip>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

#define GLOG_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

class TimeUtil {
 public:
  // @brief: UNIX timestamp to GPS timestamp, in seconds.
  static double unix2gps(double unix_time) {
    double gps_time = unix_time - UNIX_GPS_DIFF;
    if (unix_time < LEAP_SECOND_TIMESTAMP) {
      gps_time -= 1.0;
    }
    return gps_time;
  }

  // @brief: GPS timestamp to UNIX timestamp, in seconds.
  static double gps2unix(double gps_time) {
    double unix_time = gps_time + UNIX_GPS_DIFF;
    if (unix_time + 1 < LEAP_SECOND_TIMESTAMP) {
      unix_time += 1.0;
    }
    return unix_time;
  }

  static double get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const double timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
    return timestamp / 1000000;
  }

 private:
  // unix timestamp(1970.01.01) is different from gps timestamp(1980.01.06)
  static const int UNIX_GPS_DIFF = 315964782;
  // unix timestamp(2016.12.31 23:59:59(60) UTC/GMT)
  static const int LEAP_SECOND_TIMESTAMP = 1483228799;

  DISALLOW_COPY_AND_ASSIGN(TimeUtil);
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_LIB_BASE_TIME_UTIL_H
