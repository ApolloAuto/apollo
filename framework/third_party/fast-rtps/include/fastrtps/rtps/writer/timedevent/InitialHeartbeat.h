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
 * @file InitialHeartbeat.h
 *
 */

#ifndef INITIALHEARTBEAT_H_
#define INITIALHEARTBEAT_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include <fastrtps/rtps/resources/TimedEvent.h>
#include <fastrtps/rtps/common/CDRMessage_t.h>
#include <fastrtps/rtps/common/Guid.h>
#include <fastrtps/rtps/common/Locator.h>
#include <fastrtps/rtps/messages/RTPSMessageGroup.h>

namespace eprosima {
namespace fastrtps{
namespace rtps{

class ReaderProxy;


/**
 * InitialHeartbeat class, controls the initial send operation of HB.
 * @ingroup WRITER_MODULE
 */
class InitialHeartbeat: public TimedEvent
{
    public:
        /**
         *
         * @param p_RP
         * @param interval
         */
        InitialHeartbeat(ReaderProxy* rp, double interval);
        virtual ~InitialHeartbeat();

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
        ReaderProxy* rp_;
};

}
}
} /* namespace eprosima */
#endif
#endif /* INITIALHEARTBEAT_H_ */
