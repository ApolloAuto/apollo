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
 * @file ParticipantProxyData.h
 *
 */

#ifndef PARTICIPANTPROXYDATA_H_
#define PARTICIPANTPROXYDATA_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include <mutex>
#include "../../../qos/QosList.h"
#include "../../../qos/ParameterList.h"

#include "../../attributes/WriterAttributes.h"
#include "../../attributes/ReaderAttributes.h"
#include "../../common/Token.h"

#define DISCOVERY_PARTICIPANT_DATA_MAX_SIZE 5000
#define DISCOVERY_TOPIC_DATA_MAX_SIZE 500
#define DISCOVERY_PUBLICATION_DATA_MAX_SIZE 5000
#define DISCOVERY_SUBSCRIPTION_DATA_MAX_SIZE 5000
#define BUILTIN_PARTICIPANT_DATA_MAX_SIZE 100

#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER 0x00000001 << 0;
#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR 0x00000001 << 1;
#define DISC_BUILTIN_ENDPOINT_PUBLICATION_ANNOUNCER 0x00000001 << 2;
#define DISC_BUILTIN_ENDPOINT_PUBLICATION_DETECTOR 0x00000001 << 3;
#define DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_ANNOUNCER 0x00000001 << 4;
#define DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_DETECTOR 0x00000001 << 5;
#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_PROXY_ANNOUNCER 0x00000001 << 6;
#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_PROXY_DETECTOR 0x00000001 << 7;
#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_STATE_ANNOUNCER 0x00000001 << 8;
#define DISC_BUILTIN_ENDPOINT_PARTICIPANT_STATE_DETECTOR 0x00000001 << 9;
#define BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER 0x00000001 << 10;
#define BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER 0x00000001 << 11;

namespace eprosima {
namespace fastrtps{
namespace rtps {

struct CDRMessage_t;
class PDPSimple;
class RemoteParticipantLeaseDuration;
class RTPSParticipantImpl;
class ReaderProxyData;
class WriterProxyData;

/**
* ParticipantProxyData class is used to store and convert the information Participants send to each other during the PDP phase.
*@ingroup BUILTIN_MODULE
*/
class ParticipantProxyData
{
    public:

        ParticipantProxyData();
        virtual ~ParticipantProxyData();

        //!Protocol version
        ProtocolVersion_t m_protocolVersion;
        //!GUID
        GUID_t m_guid;
        //!Vendor ID
        VendorId_t m_VendorId;
        //!Expects Inline QOS.
        bool m_expectsInlineQos;
        //!Available builtin endpoints
        BuiltinEndpointSet_t m_availableBuiltinEndpoints;
        //!Metatraffic unicast locator list
        LocatorList_t m_metatrafficUnicastLocatorList;
        //!Metatraffic multicast locator list
        LocatorList_t m_metatrafficMulticastLocatorList;
        //!Default unicast locator list
        LocatorList_t m_defaultUnicastLocatorList;
        //!Default multicast locator list
        LocatorList_t m_defaultMulticastLocatorList;
        //!Manual liveliness count
        Count_t m_manualLivelinessCount;
        //!Participant name
        std::string m_participantName;
        //!
        InstanceHandle_t m_key;
        //!
        Duration_t m_leaseDuration;
        //!
        IdentityToken identity_token_;
        //!
        bool isAlive;
        //!
        QosList_t m_QosList;
        //!
        ParameterPropertyList_t m_properties;
        //!
        std::vector<octet> m_userData;
        //!
        bool m_hasChanged;
        //!
        RemoteParticipantLeaseDuration* mp_leaseDurationTimer;
        //!
        std::vector<ReaderProxyData*> m_readers;
        //!
        std::vector<WriterProxyData*> m_writers;
        //!
        std::vector<RemoteReaderAttributes> m_builtinReaders;
        //!
        std::vector<RemoteWriterAttributes> m_builtinWriters;

        std::recursive_mutex* mp_mutex;
        /**
         * Initialize the object with the data of the lcoal RTPSParticipant.
         * @param part Pointer to the RTPSParticipant.
         * @param pdp Pointer to the PDPSimple object.
         * @return True if correctly initialized.
         */
        bool initializeData(RTPSParticipantImpl* part, PDPSimple* pdp);
        /**
         * Update the data.
         * @param pdata Object to copy the data from
         * @return True on success
         */
        bool updateData(ParticipantProxyData& pdata);
        /**
         * Convert information to parameter list.
         * @return True on success
         */
        bool toParameterList();
        /**
         * Read the parameter list from a recevied CDRMessage_t
         * @return True on success
         */
        bool readFromCDRMessage(CDRMessage_t* msg);
        //!Clear the data (restore to default state.)
        void clear();
        /**
         * Copy the data from another object.
         * @param pdata Object to copy the data from
         */
        void copy(ParticipantProxyData& pdata);
};

}
} /* namespace rtps */
} /* namespace eprosima */

#endif
#endif /* RTPSParticipantPROXYDATA_H_ */
