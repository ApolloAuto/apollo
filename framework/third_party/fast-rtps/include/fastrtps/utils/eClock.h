// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file eClock.h
 *
 */

#ifndef ECLOCK_H_
#define ECLOCK_H_

#if defined(_WIN32)
#include <time.h>
#include <windows.h> 
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif
 
struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};
 

#else
#include <sys/time.h>
#include <chrono>

#endif

#include "../rtps/common/Time_t.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps{


/**
 * Class eClock used to obtain the time and to sleep some processes.
 * Time measured since 1970.
 * @ingroup UTILITIES_MODULE
 */
class RTPS_DllAPI eClock {
public:
	eClock();
	virtual ~eClock();
	//!Seconds from 1900 to 1970, initialized to 0 to comply with RTPS 2.2
	int32_t m_seconds_from_1900_to_1970;
	//!Difference from UTC in seconds
	int32_t m_utc_seconds_diff;
	
	/**
	* Fill a Time_t with the current time
	* @param now Pointer to a Time_t instance to fill with the current time
	* @return true on success
	*/
	bool setTimeNow(Time_t* now);
	
	/**
	* Method to start measuring an interval in us.
	*/
	void intervalStart();
	
	/**
	* Method to finish measuring an interval in us.
	* @return Time of the interval in us
	*/
	uint64_t intervalEnd();
	
	/**
	* Put the current thread to sleep.
	* @param milliseconds Time to sleep
	*/
	static void my_sleep(uint32_t milliseconds);
	
#if defined(_WIN32)
	FILETIME ft;
	unsigned long long ftlong;
	FILETIME ft1,ft2;
	LARGE_INTEGER freq;
	LARGE_INTEGER li1,li2;
#else
	timeval m_now;
	timeval m_interval1,m_interval2;
#endif
};

} /* namespace rtps */
} /* namespace eprosima */

#endif /* CLOCK_H_ */
