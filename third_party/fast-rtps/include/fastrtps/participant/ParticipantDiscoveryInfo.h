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
 * @file ParticipantDiscoveryInfo.h
 *
 */

#ifndef PARTICIPANTDISCOVERYINFO_H_
#define PARTICIPANTDISCOVERYINFO_H_

#include "../rtps/participant/RTPSParticipantDiscoveryInfo.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps {

/**
 * Class ParticipantDiscoveryInfo, provided to the user with information regarding a Discovered Participant.
 * @ingroup FASTRTPS_MODULE
 */
class ParticipantDiscoveryInfo
{
    public:
        ParticipantDiscoveryInfo(){}
        virtual ~ParticipantDiscoveryInfo(){}
        //!RTPSParticipantAttributes of the discovered participant.
        RTPSParticipantDiscoveryInfo rtps;
};

#if HAVE_SECURITY
class ParticipantAuthenticationInfo
{
    public:

        ParticipantAuthenticationInfo() {}

        virtual ~ParticipantAuthenticationInfo() {}

        RTPSParticipantAuthenticationInfo rtps;
};
#endif

} /* namespace fastrtps */
} /* namespace eprosima */

#endif /* PARTICIPANTDISCOVERYINFO_H_ */
