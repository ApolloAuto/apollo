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
 * @file ResourceEvent.h
 *
 */

#ifndef RESOURCEEVENT_H_
#define RESOURCEEVENT_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <thread>
#include <asio.hpp>

namespace eprosima {
namespace fastrtps{
namespace rtps {

class RTPSParticipantImpl;

/**
 * Class ResourceEvent used to manage the temporal events.
 *@ingroup MANAGEMENT_MODULE
 */
class ResourceEvent {
public:
	ResourceEvent();
	virtual ~ResourceEvent();

    /**
    * Method to initialize the thread.
    * @param p
    */
    void init_thread(RTPSParticipantImpl*p);

	/**
	* Get the associated IO service
	* @return Associated IO service
	*/
	asio::io_service& getIOService() { return *mp_io_service; }

    std::thread& getThread() { return *mp_b_thread; }

private:

	//!Thread
	std::thread* mp_b_thread;
	//!IO service
	asio::io_service* mp_io_service;
	//!
	void * mp_work;

	/**
	 * Task to announce the correctness of the thread.
	 */
	void announce_thread();

	//!Method to run the tasks
	void run_io_service();

	//!Pointer to the RTPSParticipantImpl.
	RTPSParticipantImpl* mp_RTPSParticipantImpl;
};
}
}
} /* namespace eprosima */
#endif
#endif /* RESOURCEEVENT_H_ */
