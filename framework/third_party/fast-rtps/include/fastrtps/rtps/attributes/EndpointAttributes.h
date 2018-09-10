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
 * @file EndpointAttributes.h
 */

#ifndef ENDPOINTATTRIBUTES_H_
#define ENDPOINTATTRIBUTES_H_

#include "../common/Types.h"
#include "../common/Locator.h"
#include "PropertyPolicy.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
 * Structure EndpointAttributes, describing the attributes associated with an RTPS Endpoint.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class EndpointAttributes
{
public:
    EndpointAttributes()
    {
        topicKind = NO_KEY;
        reliabilityKind = BEST_EFFORT;
        durabilityKind = VOLATILE;
        m_userDefinedID = -1;
        m_entityID = -1;
        endpointKind = WRITER;
    };
    virtual ~EndpointAttributes(){};
    //!Endpoint kind, default value WRITER
    EndpointKind_t endpointKind;
    //!Topic kind, default value NO_KEY
    TopicKind_t topicKind;
    //!Reliability kind, default value BEST_EFFORT
    ReliabilityKind_t reliabilityKind;
    //!Durability kind, default value VOLATILE
    DurabilityKind_t durabilityKind;
    //!Unicast locator list
    LocatorList_t unicastLocatorList;
    //!Multicast locator list
    LocatorList_t multicastLocatorList;
    LocatorList_t outLocatorList;
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
    inline void setUserDefinedID(uint8_t id){m_userDefinedID = id;};

    /**
     * Set the entity ID
     * @param id Entity ID to be set
     */
    inline void setEntityID(uint8_t id){m_entityID = id;};

private:
    //!User Defined ID, used for StaticEndpointDiscovery, default value -1.
    int16_t m_userDefinedID;
    //!Entity ID, if the user want to specify the EntityID of the enpoint, default value -1.
    int16_t m_entityID;
};
}
} /* namespace rtps */
} /* namespace eprosima */

#endif /*  */
