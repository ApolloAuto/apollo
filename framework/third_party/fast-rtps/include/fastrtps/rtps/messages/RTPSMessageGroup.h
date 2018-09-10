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
 * @file RTPSMessageGroup.h
 *
 */

#ifndef RTPSMESSAGEGROUP_H_
#define RTPSMESSAGEGROUP_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "../messages/RTPSMessageCreator.h"
#include "../../qos/ParameterList.h"
#include <fastrtps/rtps/common/FragmentNumber.h>

#include <vector>
#include <cassert>

namespace eprosima {
namespace fastrtps{
namespace rtps {

class RTPSParticipantImpl;
class Endpoint;

/**
 * Class RTPSMessageGroup_t that contains the messages used to send multiples changes as one message.
 * @ingroup WRITER_MODULE
 */
class RTPSMessageGroup_t
{
    public:

        RTPSMessageGroup_t(uint32_t payload, GuidPrefix_t participant_guid):
            rtpsmsg_submessage_(payload),
            rtpsmsg_fullmsg_(payload)
#if HAVE_SECURITY
            , rtpsmsg_encrypt_(payload)
#endif
        {
            CDRMessage::initCDRMsg(&rtpsmsg_fullmsg_);
            RTPSMessageCreator::addHeader(&rtpsmsg_fullmsg_, participant_guid);
        }

        CDRMessage_t rtpsmsg_submessage_;

        CDRMessage_t rtpsmsg_fullmsg_;

#if HAVE_SECURITY
        CDRMessage_t rtpsmsg_encrypt_;
#endif
};

class RTPSWriter;

/**
 * RTPSMessageGroup Class used to construct a RTPS message.
 * @ingroup WRITER_MODULE
 */
class RTPSMessageGroup
{
    public:

        enum ENDPOINT_TYPE
        {
            WRITER,
            READER
        };

        RTPSMessageGroup(RTPSParticipantImpl* participant, Endpoint* endpoint, ENDPOINT_TYPE,
                RTPSMessageGroup_t& msg_group);

        ~RTPSMessageGroup();

        bool add_data(const CacheChange_t& change, const std::vector<GUID_t>& remote_readers,
                const LocatorList_t& locators, bool expectsInlineQos);

        bool add_data_frag(const CacheChange_t& change, const uint32_t fragment_number,
                const std::vector<GUID_t>& remote_readers, const LocatorList_t& locators,
                bool expectsInlineQos);

        bool add_heartbeat(const std::vector<GUID_t>& remote_readers, const SequenceNumber_t& firstSN,
                const SequenceNumber_t& lastSN, Count_t count,
                bool isFinal, bool livelinessFlag, const LocatorList_t& locators);

        bool add_gap(std::set<SequenceNumber_t>& changesSeqNum, const std::vector<GUID_t>& remote_readers,
                const LocatorList_t& locators);

        bool add_acknack(const GUID_t& remote_writer, SequenceNumberSet_t& SNSet,
                int32_t count, bool finalFlag, const LocatorList_t& locators);

        bool add_nackfrag(const GUID_t& remote_writer, SequenceNumber_t& writerSN,
                FragmentNumberSet_t fnState, int32_t count, const LocatorList_t locators);

        uint32_t get_current_bytes_processed() { return currentBytesSent_ + full_msg_->length; }

    private:

        void reset_to_header();

        bool check_preconditions(const LocatorList_t& locator_list,
                const std::vector<GuidPrefix_t>& remote_participants) const;

        void flush_and_reset(const LocatorList_t& locator_list,
                std::vector<GuidPrefix_t>&& remote_participants);

        void flush();

        void send();

        void check_and_maybe_flush(const LocatorList_t& locator_list,
                const std::vector<GUID_t>& remote_endpoints);

        bool insert_submessage(const std::vector<GUID_t>& remote_endpoints);

        bool add_info_dst_in_buffer(CDRMessage_t* buffer, const std::vector<GUID_t>& remote_endpoints);

        bool add_info_ts_in_buffer(const std::vector<GUID_t>& remote_readers);

        RTPSParticipantImpl* participant_;

        Endpoint* endpoint_;

        CDRMessage_t* full_msg_;

        CDRMessage_t* submessage_msg_;

        uint32_t currentBytesSent_;

#if HAVE_SECURITY
        ENDPOINT_TYPE type_;

        CDRMessage_t* encrypt_msg_;
#endif

        LocatorList_t current_locators_;

        GuidPrefix_t current_dst_;

        std::vector<GuidPrefix_t> current_remote_participants_;
};

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

#endif
#endif /* RTPSMESSAGEGROUP_H_ */
