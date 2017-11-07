#ifndef IDL_CAR_TIMER_H
#define IDL_CAR_TIMER_H

#include "boost/date_time/posix_time/posix_time.hpp"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The timer to measure the time has elapsed. The accuracy is milisecond. */
class Timer {
public:
    /**@brief The constructor. */
    Timer();
    /**@brief Start the timer. */
    void start();
    /**@brief End the timer. This function will automatically start a new timer at the end of function call.
     * <title> The title in the output message. Use NULL if there is no title. */
    void end(const char * title);
private:
    boost::posix_time::ptime _start_time;
    boost::posix_time::ptime _end_time;
};

class TimeAccumulator {
public:
    /**@brief The constructor. */
    TimeAccumulator();
    /**@brief Start the timer. */
    void start();
    /**@brief End the timer and print a message. Use NULL if no message. */
    void end(const char * title = NULL);
    /**@brief Clear the accumulator. */
    void clear();
    /**@brief Get the totol duration of the timer.
     * <title> The duration will be output to std::cout. The title is message title. Use NULL if no title.
     */
    void get_duration(const char * title);
private:
    boost::posix_time::ptime _start_time;
    boost::posix_time::time_duration _duration;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // IDL_CAR_TIMER_H
