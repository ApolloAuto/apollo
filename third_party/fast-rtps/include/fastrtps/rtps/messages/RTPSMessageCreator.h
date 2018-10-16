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
 * @file RTPSMessageCreator.h
*/
#ifndef CDRMESSAGECREATOR_H_
#define CDRMESSAGECREATOR_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../common/CDRMessage_t.h"
#include "../common/Guid.h"
#include "../common/SequenceNumber.h"
#include "../common/FragmentNumber.h"
#include "../common/CacheChange.h"

namespace eprosima {
namespace fastrtps{

class ParameterList_t;

namespace rtps{

/**
 * @brief Class RTPSMessageCreator, allows the generation of serialized CDR RTPS Messages.
 * @ingroup MANAGEMENT_MODULE
 */
class RTPSMessageCreator
{
    public:

        RTPSMessageCreator();
        virtual ~RTPSMessageCreator();

        //!
        CDRMessage_t rtpsmc_submsgElem;
        //!
        CDRMessage_t rtpsmc_submsgHeader;



        /**
         * Create a Header to the serialized message.
         * @param msg Pointer to the Message.
         * @param Prefix RTPSParticipant prefix of the message.
         * @param version Protocol version.
         * @param vendorId Vendor Id.
         * @return True if correct.
         */
        static bool addHeader(CDRMessage_t*msg ,const GuidPrefix_t& Prefix,ProtocolVersion_t version,VendorId_t vendorId);

        /**
         * Create a Header to the serialized message.
         * @param msg Pointer to the Message.
         * @param Prefix RTPSParticipant prefix of the message.
         * @return True if correct.
         */
        static bool addHeader(CDRMessage_t*msg ,const GuidPrefix_t& Prefix);

        /**
         * Create SubmessageHeader.
         * @param msg Pointer to the CDRMessage.
         * @param id SubMessage Id.
         * @param flags Submessage flags.
         * @param size Submessage size.
         * @return True if correct.
         */
        static bool addSubmessageHeader(CDRMessage_t* msg,octet id,octet flags,uint16_t size);


        /** @name CDR messages creation methods.
         * These methods create a CDR message for different types
         * Depending on the function a complete message (with RTPS Header is created) or only the submessage.
         * @param[out] msg Pointer to where the message is going to be created and stored.
         * @param[in] guidPrefix Guid Prefix of the RTPSParticipant.
         * @param[in] param Different parameters depending on the message.
         * @return True if correct.
         */

        /// @{

        static bool addMessageData(CDRMessage_t* msg, GuidPrefix_t& guidprefix, const CacheChange_t* change,
                TopicKind_t topicKind, const EntityId_t& readerId, bool expectsInlineQos, ParameterList_t* inlineQos);
        static bool addSubmessageData(CDRMessage_t* msg, const CacheChange_t* change,
                TopicKind_t topicKind, const EntityId_t& readerId, bool expectsInlineQos, ParameterList_t* inlineQos);

        static bool addMessageDataFrag(CDRMessage_t* msg, GuidPrefix_t& guidprefix, const CacheChange_t* change, uint32_t fragment_number,
                TopicKind_t topicKind, const EntityId_t& readerId, bool expectsInlineQos, ParameterList_t* inlineQos);
        static bool addSubmessageDataFrag(CDRMessage_t* msg, const CacheChange_t* change, uint32_t fragment_number,
                uint32_t sample_size, TopicKind_t topicKind, const EntityId_t& readerId, bool expectsInlineQos,
                ParameterList_t* inlineQos);

        static bool addMessageGap(CDRMessage_t* msg, const GuidPrefix_t& guidprefix, const GuidPrefix_t& remoteGuidPrefix,
                const SequenceNumber_t& seqNumFirst, const SequenceNumberSet_t& seqNumList,const EntityId_t& readerId,const EntityId_t& writerId);
        static bool addSubmessageGap(CDRMessage_t* msg, const SequenceNumber_t& seqNumFirst, const SequenceNumberSet_t& seqNumList,const EntityId_t& readerId,const EntityId_t& writerId);

        static bool addMessageHeartbeat(CDRMessage_t* msg, const GuidPrefix_t& guidprefix,
                const EntityId_t& readerId, const EntityId_t& writerId,
                const SequenceNumber_t& firstSN, const SequenceNumber_t& lastSN,
                Count_t count, bool isFinal, bool livelinessFlag);
        static bool addMessageHeartbeat(CDRMessage_t* msg,const GuidPrefix_t& guidprefix,
                const GuidPrefix_t& remoteGuidprefix, const EntityId_t& readerId,
                const EntityId_t& writerId, const SequenceNumber_t& firstSN,
                const SequenceNumber_t& lastSN, Count_t count, bool isFinal, bool livelinessFlag);
        static bool addSubmessageHeartbeat(CDRMessage_t* msg, const EntityId_t& readerId, const EntityId_t& writerId,
                const SequenceNumber_t& firstSN, const SequenceNumber_t& lastSN,
                Count_t count, bool isFinal, bool livelinessFlag);

        static bool addMessageHeartbeatFrag(CDRMessage_t* msg, const GuidPrefix_t& guidprefix, const EntityId_t& readerId, const EntityId_t& writerId,
                SequenceNumber_t& firstSN, FragmentNumber_t& lastFN, Count_t count);
        static bool addSubmessageHeartbeatFrag(CDRMessage_t* msg, const EntityId_t& readerId, const EntityId_t& writerId,
                SequenceNumber_t& firstSN, FragmentNumber_t& lastFN, Count_t count);

        static bool addMessageAcknack(CDRMessage_t* msg,const GuidPrefix_t& guidprefix, const GuidPrefix_t& remoteGuidPrefix,
                const EntityId_t& readerId,const EntityId_t& writerId,SequenceNumberSet_t& SNSet,int32_t count,bool finalFlag);
        static bool addSubmessageAcknack(CDRMessage_t* msg,
                const EntityId_t& readerId,const EntityId_t& writerId,SequenceNumberSet_t& SNSet,int32_t count,bool finalFlag);

        static bool addMessageNackFrag(CDRMessage_t* msg, const GuidPrefix_t& guidprefix, const GuidPrefix_t& remoteGuidPrefix,
                const EntityId_t& readerId, const EntityId_t& writerId, SequenceNumber_t& writerSN, FragmentNumberSet_t fnState, int32_t count);
        static bool addSubmessageNackFrag(CDRMessage_t* msg,
                const EntityId_t& readerId, const EntityId_t& writerId, SequenceNumber_t& writerSN, FragmentNumberSet_t fnState, int32_t count);

        static bool addSubmessageInfoTS(CDRMessage_t* msg,Time_t& time,bool invalidateFlag);
        static bool addSubmessageInfoTS_Now(CDRMessage_t* msg,bool invalidateFlag);

        static bool addSubmessageInfoDST(CDRMessage_t* msg, GuidPrefix_t guidP);
};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* CDRMESSAGECREATOR_H_ */
