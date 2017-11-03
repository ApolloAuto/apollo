#include "modules/localization/msf/common/util/timer.h"
#include "modules/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

Timer::Timer() {

}

void Timer::start() {
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void Timer::end(const char * title) {
    _end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration dt = _end_time - _start_time;
    if (title) {
        AINFO << title << " Elapsed time: " << dt.seconds() << "s " <<
                 (dt.total_milliseconds() - dt.seconds()*1000) << "ms";
    } else {
        AINFO << " Elapsed time: " << dt.seconds() << "s " <<
                 (dt.total_milliseconds() - dt.seconds()*1000) << "ms";
    }
    _start_time = boost::posix_time::microsec_clock::local_time();
}

TimeAccumulator::TimeAccumulator() {

}

void TimeAccumulator::start() {
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::end(const char * title) {
    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration dt = end_time - _start_time;
    _duration += dt;
    if (title) {
        AINFO << title << " Elapsed time: " << dt.seconds() << "s " <<
                 (dt.total_milliseconds() - dt.seconds()*1000) << "ms";
    }
    _start_time = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::clear() {
    _duration = boost::posix_time::time_duration();
}

void TimeAccumulator::get_duration(const char * title) {
    if (title) {
        AINFO << title << " Total elapsed time: " << _duration.seconds() << "s " <<
                 (_duration.total_milliseconds() - _duration.seconds()*1000) << "ms";
    } else {
        AINFO << " Total elapsed time: " << _duration.seconds() << "s " <<
                 (_duration.total_milliseconds() - _duration.seconds()*1000) << "ms";
    }
}

} // namespace msf
} // namespace localization
} // namespace apollo
