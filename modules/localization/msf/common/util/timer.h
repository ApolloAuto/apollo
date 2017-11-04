/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_LOCALIZATION_MSF_COMMON_TIMER_H_
#define MODULES_LOCALIZATION_MSF_COMMON_TIMER_H_

#include "boost/date_time/posix_time/posix_time.hpp"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The timer to measure the time has elapsed. 
  * The accuracy is milisecond. */
class Timer {
 public:
  /**@brief The constructor. */
  Timer();
  /**@brief Start the timer. */
  void start();
  /**@brief End the timer. 
   * This function will automatically start a new timer
   * at the end of function call.
   * <title> The title in the output message.
   * Use NULL if there is no title. */
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
   * <title> The duration will be output to std::cout.
   * The title is message title. Use NULL if no title.
   */
  void get_duration(const char * title);
 private:
  boost::posix_time::ptime _start_time;
  boost::posix_time::time_duration _duration;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_COMMON_TIMER_H_
