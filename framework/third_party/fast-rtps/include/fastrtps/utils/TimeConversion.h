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
 * @file TimeConversion.h
 *
 */

#ifndef TIMECONVERSION_H_
#define TIMECONVERSION_H_

#include <cstdint>
#include "../rtps/common/Time_t.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {


namespace TimeConv{

/**
* Convert Time_t to seconds as a double
*/ 
inline double Time_t2SecondsDouble(const Time_t& t)
{
	return (double)t.seconds + (double)(t.fraction/pow(2.0,32));
}

/**
* Convert Time_t to seconds as an int64
*/ 
inline int64_t Time_t2MicroSecondsInt64(const Time_t& t)
{
	return (int64_t)(t.fraction/pow(2.0,32)*pow(10.0,6))+t.seconds*(int64_t)pow(10.0,6);
}

/**
* Convert Time_t to microseconds as a double
*/ 
inline double Time_t2MicroSecondsDouble(const Time_t& t)
{
	return ((double)t.fraction/pow(2.0,32)*pow(10.0,6))+(double)t.seconds*pow(10.0,6);
}

/**
* Convert Time_t to milliseconds as an int64
*/ 
inline int64_t Time_t2MilliSecondsInt64(const Time_t& t)
{
	return (int64_t)(t.fraction/pow(2.0,32)*pow(10.0,3))+t.seconds*(int64_t)pow(10.0,3);
}

/**
* Convert Time_t to milliseconds as a double
*/ 
inline double Time_t2MilliSecondsDouble(const Time_t& t)
{
	return ((double)t.fraction/pow(2.0,32)*pow(10.0,3))+(double)t.seconds*pow(10.0,3);
}

/**
* Convert milliseconds to Time_t
*/ 
inline Time_t MilliSeconds2Time_t(double millisec)
{
	Time_t t;
	t.seconds = (int32_t)(millisec/pow(10.0,3));
	t.fraction = (uint32_t)((millisec-(double)t.seconds*pow(10.0,3))/pow(10.0,3)*pow(2.0,32));
	return t;
}

/**
* Convert microseconds to Time_t
*/
inline Time_t MicroSeconds2Time_t(double microsec)
{
	Time_t t;
	t.seconds = (int32_t)(microsec/pow(10.0,6));
	t.fraction = (uint32_t)((microsec-(double)t.seconds*pow(10.0,6))/pow(10.0,6)*pow(2.0,32));
	return t;
}

/**
* Convert seconds to Time_t
*/
inline Time_t Seconds2Time_t(double seconds)
{
	Time_t t;
	t.seconds = (int32_t)seconds;
	t.fraction = (uint32_t)((seconds-(double)t.seconds)*pow(2.0,32));
	return t;
}

/**
* Get the absolute difference between two Time_t in milliseconds as double
*/
inline double Time_tAbsDiff2DoubleMillisec(const Time_t& t1,const Time_t& t2)
{
	double result = 0;
	result +=(double)abs((t2.seconds-t1.seconds)*1000);
	result +=(double)std::abs((t2.fraction-t1.fraction)/pow(2.0,32)*1000);
	return result;
}

//! Create a random Time_t that is millisec + [-randoff,randoff]
inline Time_t MilliSecondsWithRandOffset2Time_t(double millisec, double randoff)
{
	randoff = std::abs(randoff);
	millisec = millisec + (-randoff) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(2*randoff)));
	return MilliSeconds2Time_t(millisec);
}
//! Create a random Time_t that is microsec + [-randoff,randoff]
inline Time_t MicroSecondsWithRandOffset2Time_t(double microsec, double randoff)
{
	randoff = std::abs(randoff);
	microsec = microsec + (-randoff) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(2*randoff)));
	return MicroSeconds2Time_t(microsec);
}
//! Create a random Time_t that is sec + [-randoff,randoff]
inline Time_t SecondsWithRandOffset2Time_t(double sec, double randoff)
{
	randoff = std::abs(randoff);
	sec = sec + (-randoff) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(2*randoff)));
	return Seconds2Time_t(sec);
}

}
}
} /* namespace rtps */
} /* namespace eprosima */

#endif /* TIMECONVERSION_H_ */
