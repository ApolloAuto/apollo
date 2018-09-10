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
 * @file TimedEvent.h
 *
 */

#ifndef TIMEDEVENT_H_
#define TIMEDEVENT_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include <thread>
#include <cstdint>
#include <asio.hpp>
#include "../common/Time_t.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {

class TimedEventImpl;

/**
 * Timed Event class used to define any timed events.
 * @ingroup MANAGEMENT_MODULE
 */
class TimedEvent {
public:

	/**
	* Enum representing event statuses
	*/
	enum EventCode
	{
		EVENT_SUCCESS,
		EVENT_ABORT,
		EVENT_MSG
	};

    enum AUTODESTRUCTION_MODE
    {
        NONE,
        ON_SUCCESS,
        ALLWAYS
    };
	
	/**
	* @param service IO service to run the event.
   * @param event_thread starting thread for identification.
	* @param milliseconds Interval of the timedEvent.
   * @param autodestruction Self-destruct mode flag.
	*/
    TimedEvent(asio::io_service &service, const std::thread& event_thread, double milliseconds, TimedEvent::AUTODESTRUCTION_MODE autodestruction = TimedEvent::NONE);
	virtual ~TimedEvent();
	
	/**
	* Method invoked when the event occurs. Abstract method.
	*
	* @param code Code representing the status of the event
	* @param msg Message associated to the event. It can be nullptr.
	*/
	virtual void event(EventCode code, const char* msg) = 0;

    void cancel_timer();
	
	//!Method to restart the timer.
	void restart_timer();
	
	/**
	* Update event interval.
	* When updating the interval, the timer is not restarted and the new interval will only be used the next time you call restart_timer().
	*
	* @param inter New interval for the timedEvent
	* @return true on success
	*/
	bool update_interval(const Duration_t& inter);
	
	/**
	* Update event interval.
	* When updating the interval, the timer is not restarted and the new interval will only be used the next time you call restart_timer().
	*
	* @param time_millisec New interval for the timedEvent
	* @return true on success
	*/
	bool update_interval_millisec(double time_millisec);
	
	/**
	* Get the milliseconds interval
	* @return Mulliseconds interval
	*/
    double getIntervalMilliSec();
	
	/**
	* Get the remaining milliseconds for the timer to expire
	* @return Remaining milliseconds for the timer to expire
	*/
    double getRemainingTimeMilliSec();

    protected:

    void destroy();

private:
	TimedEventImpl* mp_impl;
};
}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif
