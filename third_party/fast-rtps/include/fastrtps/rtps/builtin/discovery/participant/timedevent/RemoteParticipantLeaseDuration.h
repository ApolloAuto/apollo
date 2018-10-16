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
 * @file RemoteParticipantLeaseDuration.h
 *
*/

#ifndef RTPSPARTICIPANTLEASEDURATION_H_
#define RTPSPARTICIPANTLEASEDURATION_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "fastrtps/rtps/resources/TimedEvent.h"


namespace eprosima {
namespace fastrtps{
namespace rtps {

class PDPSimple;
class ParticipantProxyData;

/**
 * Class RemoteRTPSParticipantLeaseDuration, TimedEvent designed to remove a
 * remote RTPSParticipant and all its Readers and Writers from the local RTPSParticipant if it fails to
 * announce its liveliness each leaseDuration period.
 *@ingroup DISCOVERY_MODULE
 */
class RemoteParticipantLeaseDuration:public TimedEvent
{
public:
	/**
	 * Constructor
	 * @param p_SPDP Pointer to the PDPSimple object.
	 * @param pdata Pointer to the ParticipantProxyData associated with this TimedEvent.
	 * @param interval Interval in ms.
	 */
	RemoteParticipantLeaseDuration(PDPSimple* p_SPDP,
			ParticipantProxyData* pdata,
			double interval);
	virtual ~RemoteParticipantLeaseDuration();

 	/**
	*  Temporal event that check if the RTPSParticipant is alive, and removes it if not.
	* @param code Code representing the status of the event
	* @param msg Message associated to the event
	*/
	void event(EventCode code, const char* msg= nullptr);
	//!Pointer to the PDPSimple object.
	PDPSimple* mp_PDP;
	//!Pointer to the RTPSParticipantProxyData object that contains this temporal event.
	ParticipantProxyData* mp_participantProxyData;

};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* RTPSPARTICIPANTLEASEDURATION_H_ */
