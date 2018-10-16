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
 * @file ReaderProxyData.h
 *
 */

#ifndef READERPROXYDATA_H_
#define READERPROXYDATA_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "../../../attributes/TopicAttributes.h"
#include "../../../qos/ParameterList.h"
#include "../../../qos/ReaderQos.h"

#include "../../attributes/WriterAttributes.h"

using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{
namespace rtps {

struct CDRMessage_t;

/**
 * Class ReaderProxyData, used to represent all the information on a Reader (both local and remote) with the purpose of
 * implementing the discovery.
 * *@ingroup BUILTIN_MODULE
 */
class ReaderProxyData
{
    public:

        RTPS_DllAPI ReaderProxyData();

        RTPS_DllAPI virtual ~ReaderProxyData();

        RTPS_DllAPI ReaderProxyData(const ReaderProxyData& readerInfo);

        RTPS_DllAPI ReaderProxyData& operator=(const ReaderProxyData& readerInfo);

        RTPS_DllAPI void guid(const GUID_t& guid)
        {
            m_guid = guid;
        }

        RTPS_DllAPI void guid(GUID_t&& guid)
        {
            m_guid = std::move(guid);
        }

        RTPS_DllAPI GUID_t guid() const
        {
            return m_guid;
        }

        RTPS_DllAPI GUID_t& guid()
        {
            return m_guid;
        }

        RTPS_DllAPI void unicastLocatorList(const LocatorList_t& unicastLocatorList)
        {
            m_unicastLocatorList = unicastLocatorList;
        }

        RTPS_DllAPI void unicastLocatorList(LocatorList_t&& unicastLocatorList)
        {
            m_unicastLocatorList = std::move(unicastLocatorList);
        }

        RTPS_DllAPI LocatorList_t unicastLocatorList() const
        {
            return m_unicastLocatorList;
        }

        RTPS_DllAPI LocatorList_t& unicastLocatorList()
        {
            return m_unicastLocatorList;
        }

        RTPS_DllAPI void multicastLocatorList(const LocatorList_t& multicastLocatorList)
        {
            m_multicastLocatorList = multicastLocatorList;
        }

        RTPS_DllAPI void multicastLocatorList(LocatorList_t&& multicastLocatorList)
        {
            m_multicastLocatorList = std::move(multicastLocatorList);
        }

        RTPS_DllAPI LocatorList_t multicastLocatorList() const
        {
            return m_multicastLocatorList;
        }

        RTPS_DllAPI LocatorList_t& multicastLocatorList()
        {
            return m_multicastLocatorList;
        }

        RTPS_DllAPI void key(const InstanceHandle_t& key)
        {
            m_key = key;
        }

        RTPS_DllAPI void key(InstanceHandle_t&& key)
        {
            m_key = std::move(key);
        }

        RTPS_DllAPI InstanceHandle_t key() const
        {
            return m_key;
        }

        RTPS_DllAPI InstanceHandle_t& key()
        {
            return m_key;
        }

        RTPS_DllAPI void RTPSParticipantKey(const InstanceHandle_t& RTPSParticipantKey)
        {
            m_RTPSParticipantKey = RTPSParticipantKey;
        }

        RTPS_DllAPI void RTPSParticipantKey(InstanceHandle_t&& RTPSParticipantKey)
        {
            m_RTPSParticipantKey = std::move(RTPSParticipantKey);
        }

        RTPS_DllAPI InstanceHandle_t RTPSParticipantKey() const
        {
            return m_RTPSParticipantKey;
        }

        RTPS_DllAPI InstanceHandle_t& RTPSParticipantKey()
        {
            return m_RTPSParticipantKey;
        }

        RTPS_DllAPI void typeName(const std::string& typeName)
        {
            m_typeName = typeName;
        }

        RTPS_DllAPI void typeName(std::string&& typeName)
        {
            m_typeName = std::move(typeName);
        }

        RTPS_DllAPI std::string typeName() const
        {
            return m_typeName;
        }

        RTPS_DllAPI std::string& typeName()
        {
            return m_typeName;
        }

        RTPS_DllAPI void topicName(const std::string& topicName)
        {
            m_topicName = topicName;
        }

        RTPS_DllAPI void topicName(std::string&& topicName)
        {
            m_topicName = std::move(topicName);
        }

        RTPS_DllAPI std::string topicName() const
        {
            return m_topicName;
        }

        RTPS_DllAPI std::string& topicName()
        {
            return m_topicName;
        }

        RTPS_DllAPI void userDefinedId(uint16_t userDefinedId)
        {
            m_userDefinedId = userDefinedId;
        }

        RTPS_DllAPI uint16_t userDefinedId() const
        {
            return m_userDefinedId;
        }

        RTPS_DllAPI uint16_t& userDefinedId()
        {
            return m_userDefinedId;
        }

        RTPS_DllAPI void isAlive(bool isAlive)
        {
            m_isAlive = isAlive;
        }

        RTPS_DllAPI bool isAlive() const
        {
            return m_isAlive;
        }

        RTPS_DllAPI bool& isAlive()
        {
            return m_isAlive;
        }

        RTPS_DllAPI void topicKind(TopicKind_t topicKind)
        {
            m_topicKind = topicKind;
        }

        RTPS_DllAPI TopicKind_t topicKind() const
        {
            return m_topicKind;
        }

        RTPS_DllAPI TopicKind_t& topicKind()
        {
            return m_topicKind;
        }

        /**
         * Convert the data to a parameter list to send this information as a RTPS message.
         * @return true if correct.
         */
        bool toParameterList();
        /**
         *  Read the information from a CDRMessage_t. The position of hte message must be in the beggining on the parameter list.
         * @param msg Pointer to the message.
         * @return true on success
         */
        RTPS_DllAPI bool readFromCDRMessage(CDRMessage_t* msg);

        //!
        bool m_expectsInlineQos;
        //!Reader Qos	
        ReaderQos m_qos;
        //!Parameter list
        ParameterList_t m_parameterList;
        /**
         * Clear (put to default) the information.
         */
        void clear();
        /**
         * Update the information (only certain fields can be updated).
         * @param rdata Poitner to the object from which we are going to update.
         */
        void update(ReaderProxyData* rdata);
        /**
         * Copy ALL the information from another object.
         * @param rdata Pointer to the object from where the information must be copied.
         */
        void copy(ReaderProxyData* rdata);

        /**
         * Convert the ProxyData information to RemoteReaderAttributes object. 
         * @return Reference to the RemoteReaderAttributes object.
         */
        RemoteReaderAttributes& toRemoteReaderAttributes();

        //!Remote Attributes associated with this proxy data.
        RemoteReaderAttributes m_remoteAtt;

    private:

        //!GUID
        GUID_t m_guid;
        //!Unicast locator list
        LocatorList_t m_unicastLocatorList;
        //!Multicast locator list
        LocatorList_t m_multicastLocatorList;
        //!GUID_t of the Reader converted to InstanceHandle_t
        InstanceHandle_t m_key;
        //!GUID_t of the participant converted to InstanceHandle
        InstanceHandle_t m_RTPSParticipantKey;
        //!Type name
        std::string m_typeName;
        //!Topic name
        std::string m_topicName;
        //!User defined ID
        uint16_t m_userDefinedId;
        //!Field to indicate if the Reader is Alive.
        bool m_isAlive;
        //!Topic kind
        TopicKind_t m_topicKind;
};

}
} /* namespace rtps */
} /* namespace eprosima */

#endif
#endif /* READERPROXYDATA_H_ */
