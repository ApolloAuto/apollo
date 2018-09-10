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
 * @file RTPSParticipantListener.h
 *
 */

#ifndef RTPSPARTICIPANTLISTENER_H_
#define RTPSPARTICIPANTLISTENER_H_

#include "RTPSParticipantDiscoveryInfo.h"

namespace eprosima{
namespace fastrtps{
namespace rtps{

class RTPSParticipant;

/**
* Class RTPSParticipantListener with virtual method that the user can overload to respond to certain events.
* @ingroup RTPS_MODULE
*/
class RTPS_DllAPI RTPSParticipantListener
{
    public:

        RTPSParticipantListener(){};
        virtual ~RTPSParticipantListener(){};

        /**
         * This method is invoked when a new participant is discovered
         * @param part Discovered participant
         * @param info Discovery information of the participant
         */
        virtual void onRTPSParticipantDiscovery(RTPSParticipant* part, RTPSParticipantDiscoveryInfo info){(void)part; (void)info;}

#if HAVE_SECURITY
        virtual void onRTPSParticipantAuthentication(RTPSParticipant* part, const RTPSParticipantAuthenticationInfo& info) {(void)part; (void)info;}
#endif
};
}
}
}



#endif /* RTPSPARTICIPANTLISTENER_H_ */
