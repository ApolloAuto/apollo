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
 * @file PeriodicHeartbeat.h
 *
 */

#ifndef PERIODICHEARTBEAT_H_
#define PERIODICHEARTBEAT_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../../resources/TimedEvent.h"
#include "../../common/CDRMessage_t.h"
#include "../../messages/RTPSMessageGroup.h"

namespace eprosima {
namespace fastrtps{
namespace rtps{

class StatefulWriter;


/**
 * PeriodicHeartbeat class, controls the periodic send operation of HB.
 * @ingroup WRITER_MODULE
 */
class PeriodicHeartbeat: public TimedEvent
{
    public:
        /**
         *
         * @param p_RP
         * @param interval
         */
        PeriodicHeartbeat(StatefulWriter* p_RP,double interval);
        virtual ~PeriodicHeartbeat();

        /**
         * Method invoked when the event occurs
         *
         * @param code Code representing the status of the event
         * @param msg Message associated to the event
         */
        void event(EventCode code, const char* msg= nullptr);

        //!
        RTPSMessageGroup_t m_cdrmessages;
        //!
        StatefulWriter* mp_SFW;
};

}
}
} /* namespace eprosima */
#endif
#endif /* PERIODICHEARTBEAT_H_ */
