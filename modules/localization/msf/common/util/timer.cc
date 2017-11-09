#include "modules/localization/msf/common/util/timer.h"
#include "modules/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

Timer::Timer() {}

void Timer::Start() {
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void Timer::End(const char* title) {
  end_time_ = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration dt = end_time_ - start_time_;
  if (title) {
    AINFO << title << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  } else {
    AINFO << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  }
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

TimeAccumulator::TimeAccumulator() {}

void TimeAccumulator::Start() {
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::End(const char* title) {
  boost::posix_time::ptime end_time =
      boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration dt = end_time - start_time_;
  duration_ += dt;
  if (title) {
    AINFO << title << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  }
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::Clear() {
  duration_ = boost::posix_time::time_duration();
}

void TimeAccumulator::GetDuration(const char* title) {
  if (title) {
    AINFO << title << " Total elapsed time: " << duration_.seconds() << "s "
          << (duration_.total_milliseconds() - duration_.seconds() * 1000)
          << "ms";
  } else {
    AINFO << " Total elapsed time: " << duration_.seconds() << "s "
          << (duration_.total_milliseconds() - duration_.seconds() * 1000)
          << "ms";
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
