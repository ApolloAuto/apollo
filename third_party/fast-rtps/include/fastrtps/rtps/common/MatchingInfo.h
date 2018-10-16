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
 * @file MatchingInfo.h
 *
 */

#ifndef MATCHINGINFO_H_
#define MATCHINGINFO_H_

#include "Guid.h"

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * @enum MatchingStatus, indicates whether the matched publication/subscription method of the PublisherListener or SubscriberListener has
 * been called for a matching or a removal of a remote endpoint.
 * @ingroup COMMON_MODULE
 */
#if defined(_WIN32)
	enum RTPS_DllAPI MatchingStatus{
#else
		enum MatchingStatus{
#endif
	MATCHED_MATCHING,//!< MATCHED_MATCHING, new publisher/subscriber found
	REMOVED_MATCHING //!< REMOVED_MATCHING, publisher/subscriber removed

};

/**
 * Class MatchingInfo contains information about the matching between two endpoints.
 * @ingroup COMMON_MODULE
 */
class RTPS_DllAPI MatchingInfo
{
public:
	//!Default constructor
	MatchingInfo():status(MATCHED_MATCHING){};
	/**
	* @param stat Status
	* @param guid GUID
	*/
	MatchingInfo(MatchingStatus stat,const GUID_t&guid):status(stat),remoteEndpointGuid(guid){};
	~MatchingInfo(){};
	//!Status
	MatchingStatus status;
	//!Remote endpoint GUID
	GUID_t remoteEndpointGuid;
};
}
}
}

#endif /* MATCHINGINFO_H_ */
