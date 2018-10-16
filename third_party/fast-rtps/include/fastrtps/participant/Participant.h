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
 * @file Participant.h
 *
 */

#ifndef PARTICIPANT_H_
#define PARTICIPANT_H_

#include "../rtps/common/Guid.h"

#include "../rtps/attributes/RTPSParticipantAttributes.h"

#include "../rtps/reader/StatefulReader.h"
#include <utility>

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps{


class ParticipantImpl;
class ParticipantAttributes;

namespace rtps {
class WriterProxyData;
class ReaderProxyData;
}

/**
 * Class Participant used to group Publishers and Subscribers into a single working unit.
 * @ingroup FASTRTPS_MODULE
 */
class RTPS_DllAPI Participant {
	friend class Domain;
	friend class ParticipantImpl;
private:

	Participant();

	virtual ~Participant();

	ParticipantImpl* mp_impl;

public:

	/**
	*	Get the GUID_t of the associated RTPSParticipant.
	* @return GUID_t
	*/
	const GUID_t& getGuid()const ;

	/**
	* Get the ParticipantAttributes.
	* @return ParticipantAttributes.
	*/
	const ParticipantAttributes& getAttributes();

	/**
	* Called when using a StaticEndpointDiscovery mechanism different that the one
	* included in FastRTPS, for example when communicating with other implementations.
	* It indicates to the Participant that an Endpoint from the XML has been discovered and
	* should be activated.
	* @param partguid Participant GUID_t.
	* @param userId User defined ID as shown in the XML file.
	* @param kind EndpointKind (WRITER or READER)
	* @return True if correctly found and activated.
	*/
	bool newRemoteEndpointDiscovered(const GUID_t& partguid, uint16_t userId,
		EndpointKind_t kind);

	/**
	 * This method returns a pointer to the Endpoint Discovery Protocol Readers (when not in Static mode)
	 * SimpleEDP creates two readers, one for Publishers and one for Subscribers, and they are both returned
	 * as a std::pair of pointers. These readers in particular have modified listeners that allow a slave 
	 * listener to attach its callbach to the original one, allowing for the addition of logging elements.
	 * 
	 * @return std::pair of pointers to the EDP Readers
	 * */	
	std::pair<StatefulReader*,StatefulReader*> getEDPReaders();

	std::vector<std::string> getParticipantNames();

    bool get_remote_writer_info(const GUID_t& writerGuid, WriterProxyData& returnedInfo);

    bool get_remote_reader_info(const GUID_t& readerGuid, ReaderProxyData& returnedInfo);
};

}
} /* namespace eprosima */

#endif /* PARTICIPANT_H_ */
