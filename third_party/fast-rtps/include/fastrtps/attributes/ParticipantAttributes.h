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
 * @file ParticipantAttributes.h
 *
 */

#ifndef PARTICIPANTATTRIBUTES_H_
#define PARTICIPANTATTRIBUTES_H_

#include "../rtps/attributes/RTPSParticipantAttributes.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima{
namespace fastrtps{

/**
 * Class ParticipantAttributes, used by the user to define the attributes of a Participant.
 * The Participants in the Publisher-Subscriber layer are only a container to a RTPSParticipant, so their attributes are the same.
 * Still to maintain the equivalence this class is used to define them.
 * @ingroup FASTRTPS_ATTRIBUTES_MODULE
 */
class ParticipantAttributes
{
public:
	ParticipantAttributes(){};
	virtual ~ParticipantAttributes(){};
	//!Attributes of the associated RTPSParticipant.
	RTPSParticipantAttributes rtps;
};

}
}



#endif /* PARTICIPANTATTRIBUTES_H_ */
