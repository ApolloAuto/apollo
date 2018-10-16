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
 * @file SubscriberAttributes.h 	
 */

#ifndef SUBSCRIBERATTRIBUTES_H_
#define SUBSCRIBERATTRIBUTES_H_

#include <fastrtps/rtps/resources/ResourceManagement.h>

#include "../rtps/common/Time_t.h"
#include "../rtps/common/Locator.h"
#include "../rtps/attributes/ReaderAttributes.h"
#include "TopicAttributes.h"
#include "../qos/ReaderQos.h"
#include "../rtps/attributes/PropertyPolicy.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps {

/**
 * Class SubscriberAttributes, used by the user to define the attributes of a Subscriber.
 * @ingroup FASTRTPS_ATTRIBUTES_MODULE
 */
class SubscriberAttributes {

    public:
        SubscriberAttributes()
        {
            m_userDefinedID = -1;
            m_entityID = -1;
            expectsInlineQos = false;
            historyMemoryPolicy = PREALLOCATED_MEMORY_MODE;
        };
        virtual ~SubscriberAttributes(){};
        //!Topic Attributes
        TopicAttributes topic;
        //!Reader QOs.
        ReaderQos qos;
        //!Times for a RELIABLE Reader
        ReaderTimes times;
        //!Unicast locator list
        LocatorList_t unicastLocatorList;
        //!Multicast locator list
        LocatorList_t multicastLocatorList;
        //!Output locator list
        LocatorList_t outLocatorList;
        //!Expects Inline QOS
        bool expectsInlineQos;
        //!Underlying History memory policy
        MemoryManagementPolicy_t historyMemoryPolicy;
        PropertyPolicy properties;

        /**
         * Get the user defined ID
         * @return User defined ID
         */
        inline int16_t getUserDefinedID() const {return m_userDefinedID;}

        /**
         * Get the entity defined ID
         * @return Entity ID
         */
        inline int16_t getEntityID() const {return m_entityID;}

        /**
         * Set the user defined ID
         * @param id User defined ID to be set
         */
        inline void setUserDefinedID(uint8_t id){m_userDefinedID = id;	};

        /**
         * Set the entity ID
         * @param id Entity ID to be set
         */
        inline void setEntityID(uint8_t id){m_entityID = id;	};
    private:
        //!User Defined ID, used for StaticEndpointDiscovery, default value -1.
        int16_t m_userDefinedID;
        //!Entity ID, if the user want to specify the EntityID of the enpoint, default value -1.
        int16_t m_entityID;
};

} /* namespace fastrtps */
} /* namespace eprosima */

#endif /* SUBSCRIBERPARAMS_H_ */
