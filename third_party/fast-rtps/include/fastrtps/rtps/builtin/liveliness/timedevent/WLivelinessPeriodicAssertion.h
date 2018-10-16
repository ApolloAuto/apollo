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
 * @file WLivelinessPeriodicAssertion.h
 *
 */

#ifndef WLIVELINESSPERIODICASSERTION_H_
#define WLIVELINESSPERIODICASSERTION_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../../../../qos/QosPolicies.h"
#include "../../../resources/TimedEvent.h"
#include "../../../../qos/ParameterList.h"

using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{
namespace rtps {

class WLP;

/**
 * Class WLivelinessPeriodicAssertion, used to assert the liveliness of the writers in a RTPSParticipant.
 * @ingroup LIVELINESS_MODULE
 */
class WLivelinessPeriodicAssertion: public TimedEvent {
public:
	/**
	* Constructor
	* @param pwlp Pointer to the WLP object.
	* @param kind Kind of the periodic assertion timed event
	*/
	WLivelinessPeriodicAssertion(WLP* pwlp,LivelinessQosPolicyKind kind);
	virtual ~WLivelinessPeriodicAssertion();
	/**
	* Method invoked when the event occurs
	* @param code Code representing the status of the event
	* @param msg Message associated to the event
	*/
	void event(EventCode code, const char* msg= nullptr);
	//!Liveliness Kind that is being asserted by this object.
	LivelinessQosPolicyKind m_livelinessKind;
	//!Pointer to the WLP object.
	WLP* mp_WLP;
	//!Assert the liveliness of AUTOMATIC kind Writers.
	bool AutomaticLivelinessAssertion();
	//!Assert the liveliness of MANUAL_BY_PARTICIPANT kind writers.
	bool ManualByRTPSParticipantLivelinessAssertion();
	//!Message to store the data.
	CDRMessage_t m_msg;
	//!Instance Handle
	InstanceHandle_t m_iHandle;
	//!GuidPrefix_t
	GuidPrefix_t m_guidP;
};

} /* namespace rtps */
} /* namespace eprosima */
}
#endif
#endif /* WLIVELINESSPERIODICASSERTION_H_ */
