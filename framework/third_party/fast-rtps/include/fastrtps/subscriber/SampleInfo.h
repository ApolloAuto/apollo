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
 * @file SampleInfo.h
 */

#ifndef SAMPLEINFO_H_
#define SAMPLEINFO_H_

#include <cstdint>

#include "../fastrtps_dll.h"

#include "../rtps/common/Time_t.h"
#include "../rtps/common/InstanceHandle.h"
#include "../rtps/common/CacheChange.h"

namespace eprosima {
namespace fastrtps {

/**
 * Class SampleInfo_t with information that is provided along a sample when reading data from a Subscriber.
 * @ingroup FASTRTPS_MODULE
 */
class RTPS_DllAPI SampleInfo_t {
public:
	SampleInfo_t():sampleKind(ALIVE), ownershipStrength(0),
    sample_identity(SampleIdentity::unknown()), related_sample_identity(SampleIdentity::unknown()) {}

	virtual ~SampleInfo_t(){};
	//!Sample kind.
	ChangeKind_t sampleKind;
	//!Ownership Strength of the writer of the sample (0 if the ownership kind is set to SHARED_OWNERSHIP_QOS).
	uint16_t ownershipStrength;
	//!Source timestamp of the sample.
	Time_t sourceTimestamp;
	//!InstanceHandle of the data
	InstanceHandle_t iHandle;

    SampleIdentity sample_identity;

    SampleIdentity related_sample_identity;
};

} /* namespace  */
} /* namespace eprosima */

#endif /* SAMPLEINFO_H_ */
