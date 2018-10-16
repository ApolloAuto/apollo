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
 * @file PDPSimpleListener.h
 *
 */

#ifndef PDPSIMPLELISTENER_H_
#define PDPSIMPLELISTENER_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "../../../reader/ReaderListener.h"
#include "../../../../qos/ParameterList.h"
#include "../../data/ParticipantProxyData.h"
using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{
namespace rtps {

class PDPSimple;
class DiscoveredParticipantData;
class RTPSReader;


/**
 * Class PDPSimpleListener, specification of SubscriberListener used by the SPDP to perform the History check when a new message is received.
 * This class is implemented in order to use the same structure than with any other RTPSReader.
 *@ingroup DISCOVERY_MODULE
 */
class PDPSimpleListener: public ReaderListener {
public:
	/**
	* @param in_SPDP
	*/
	PDPSimpleListener(PDPSimple* in_SPDP) : mp_SPDP(in_SPDP)
	{
	}

	virtual ~PDPSimpleListener() {}
	//!Pointer to the associated mp_SPDP;
	PDPSimple* mp_SPDP;
	/**
	 * New added cache
	 * @param reader
	 * @param change
	 */
	void onNewCacheChangeAdded(RTPSReader* reader,const CacheChange_t* const change);
	/**
	 * Process a new added cache with this method.
	 * @return True on success
	 */
	bool newAddedCache();
	/**
	 * Get the key of a CacheChange_t
	 * @param change Pointer to the CacheChange_t
	 * @return True on success
	 */
	bool getKey(CacheChange_t* change);
	//!Temporal RTPSParticipantProxyData object used to read the messages.
	//ParticipantProxyData m_ParticipantProxyData;
	//!Auxiliary message.
	//CDRMessage_t aux_msg;
};


}
} /* namespace rtps */
} /* namespace eprosima */

#endif
#endif /* PDPSIMPLELISTENER_H_ */
