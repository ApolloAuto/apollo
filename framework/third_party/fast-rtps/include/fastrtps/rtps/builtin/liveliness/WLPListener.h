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
 * @file WLPListener.h
 *
 */

#ifndef WLPLISTENER_H_
#define WLPLISTENER_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../../reader/ReaderListener.h"

#include "../../common/Guid.h"
#include "../../common/InstanceHandle.h"
#include "../../../qos/QosPolicies.h"
#include "../../../qos/ParameterList.h"


using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{



namespace rtps {

class WLP;
class RTPSReader;
struct CacheChange_t;

/**
 * Class WLPListener that receives the liveliness messages asserting the liveliness of remote endpoints.
 * @ingroup LIVELINESS_MODULE
 */
class WLPListener: public ReaderListener {
public:
	/**
	 * Constructor
	 * @param pwlp Pointer to the WLP object.
	 */
	WLPListener(WLP* pwlp);
	virtual ~WLPListener();

	/**
	*
	* @param reader
	* @param change
	*/
	void onNewCacheChangeAdded(RTPSReader* reader,const CacheChange_t* const  change);
	/**
	* Separate the Key between the GuidPrefix_t and the liveliness Kind
	* @param key InstanceHandle_t to separate.
	* @param guidP GuidPrefix_t pointer to store the info.
	* @param liveliness Liveliness Kind Pointer.
	* @return True if correctly separated.
	*/
	bool separateKey(InstanceHandle_t& key,
			GuidPrefix_t* guidP,
			LivelinessQosPolicyKind* liveliness);
			
	/**
	* Compute the key from a CacheChange_t 
	* @param change
	*/
	bool computeKey(CacheChange_t* change);
	
	//!Auxiliary message.
	CDRMessage_t aux_msg;

private:
	WLP* mp_WLP;

};

} /* namespace rtps */
} /* namespace eprosima */
}
#endif
#endif /* WLPLISTENER_H_ */
