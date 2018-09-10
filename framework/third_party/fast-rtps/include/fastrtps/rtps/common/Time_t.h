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
 * @file Time_t.h 	
 */

#ifndef TIME_T_H_
#define TIME_T_H_
#include "../../fastrtps_dll.h"
#include <cmath>
#include <cstdint>
#include <iostream>

namespace eprosima{
namespace fastrtps{
namespace rtps{
/**
 * Structure Time_t, used to describe times.
 * @ingroup COMMON_MODULE
 */
struct RTPS_DllAPI Time_t{
	//!Seconds
	int32_t seconds;
	//!Fraction of second (1 fraction = 1/(2^32) seconds)
	uint32_t fraction;
	//! Default constructor. Sets values to zero.
	Time_t()
	{
		seconds = 0;
		fraction = 0;
	}
	/**
	* @param sec Seconds
	* @param frac Fraction of second
	*/
	Time_t(int32_t sec,uint32_t frac)
	{
		seconds = sec;
		fraction = frac;
	}
	
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
* Comparison assignment
* @param t1 First Time_t to compare
* @param t2 Second Time_t to compare
* @return True if equal
*/
static inline bool operator==(const Time_t& t1,const Time_t& t2)
{
	if(t1.seconds!=t2.seconds)
		return false;
	if(t1.fraction!=t2.fraction)
		return false;
	return true;
}

/**
* Comparison assignment
* @param t1 First Time_t to compare
* @param t2 Second Time_t to compare
* @return True if not equal
*/
static inline bool operator!=(const Time_t& t1,const Time_t& t2)
{
	if(t1.seconds!=t2.seconds)
		return true;
	if(t1.fraction!=t2.fraction)
		return true;
	return false;
}

/**
 * Checks if a Time_t is less than other.
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if the first Time_t is less than the second
 */
static inline bool operator<(const Time_t& t1,const Time_t& t2)
{
	if(t1.seconds < t2.seconds)
		return true;
	else if(t1.seconds > t2.seconds)
		return false;
	else
	{
		if(t1.fraction < t2.fraction)
			return true;
		else
			return false;
	}
}

/**
 * Checks if a Time_t is less or equal than other.
 * @param t1 First Time_t to compare
 * @param t2 Second Time_t to compare
 * @return True if the first Time_t is less or equal than the second
 */
static inline bool operator<=(const Time_t& t1,const Time_t& t2)
{
	if(t1.seconds < t2.seconds)
		return true;
	else if(t1.seconds > t2.seconds)
		return false;
	else
	{
		if(t1.fraction <= t2.fraction)
			return true;
		else
			return false;
	}
}

inline std::ostream& operator<<(std::ostream& output,const Time_t& t)
{
	return output << t.seconds<<"."<<t.fraction;
}

#endif

const Time_t c_TimeInfinite(0x7fffffff,0xffffffff);
const Time_t c_TimeZero(0,0);
const Time_t c_TimeInvalid(-1,0xffffffff);

typedef Time_t Duration_t;


}
}
}

#endif /* TIME_T_H_ */
