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

#ifndef _RTPS_SECURITY_COMMON_PARTICIPANTGENERICMESSAGE_H_
#define _RTPS_SECURITY_COMMON_PARTICIPANTGENERICMESSAGE_H_

#include "../../common/Guid.h"
#include "../../common/Token.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

class MessageIdentity
{
    public:

        MessageIdentity() : sequence_number_(0) {}

        MessageIdentity(const MessageIdentity& mi) :
            source_guid_(mi.source_guid_),
            sequence_number_(mi.sequence_number_)
        {
        }

        MessageIdentity(MessageIdentity&& mi) :
            source_guid_(std::move(mi.source_guid_)),
            sequence_number_(mi.sequence_number_)
        {
        }

        MessageIdentity& operator=(const MessageIdentity& mi)
        {
            source_guid_ = mi.source_guid_;
            sequence_number_ = mi.sequence_number_;
            return *this;
        }

        MessageIdentity& operator=(MessageIdentity&& mi)
        {
            source_guid_ = std::move(mi.source_guid_);
            sequence_number_ = mi.sequence_number_;
            return *this;
        }

        void source_guid(const GUID_t& source_guid)
        {
            source_guid_ = source_guid;
        }

        void source_guid(GUID_t&& source_guid)
        {
            source_guid_ = std::move(source_guid);
        }

        const GUID_t& source_guid() const
        {
            return source_guid_;
        }

        GUID_t& source_guid()
        {
            return source_guid_;
        }

        void sequence_number(int64_t sequence_number)
        {
            sequence_number_ = sequence_number;
        }

        int64_t sequence_number() const
        {
            return sequence_number_;
        }

        int64_t& sequence_number()
        {
            return sequence_number_;
        }

    private:

        GUID_t source_guid_;
        int64_t sequence_number_;
};

class MessageIdentityHelper
{
    public:

        static size_t serialized_size(const MessageIdentity& /*message*/, size_t current_alignment = 0)
        {
            size_t initial_alignment = current_alignment;

            current_alignment += 16;
            current_alignment +=  alignment(current_alignment, 8) + 8;

            return current_alignment - initial_alignment;
        }

    private:

        inline static size_t alignment(size_t current_alignment, size_t dataSize) { return (dataSize - (current_alignment % dataSize)) & (dataSize-1);}
};

class ParticipantGenericMessage
{
    public:

        ParticipantGenericMessage() {}

        ParticipantGenericMessage(const ParticipantGenericMessage& message) :
            message_identity_(message.message_identity_),
            related_message_identity_(message.related_message_identity_),
            destination_participant_key_(message.destination_participant_key_),
            destination_endpoint_key_(message.destination_endpoint_key_),
            source_endpoint_key_(message.source_endpoint_key_),
            message_class_id_(message.message_class_id_),
            message_data_(message.message_data_)
        {}

        ParticipantGenericMessage(ParticipantGenericMessage&& message) :
            message_identity_(std::move(message.message_identity_)),
            related_message_identity_(std::move(message.related_message_identity_)),
            destination_participant_key_(std::move(message.destination_participant_key_)),
            destination_endpoint_key_(std::move(message.destination_endpoint_key_)),
            source_endpoint_key_(std::move(message.source_endpoint_key_)),
            message_class_id_(std::move(message.message_class_id_)),
            message_data_(std::move(message.message_data_))
        {}

        ParticipantGenericMessage& operator=(const ParticipantGenericMessage& message)
        {
            message_identity_ = message.message_identity_;
            related_message_identity_ = message.related_message_identity_;
            destination_participant_key_ = message.destination_participant_key_;
            destination_endpoint_key_ = message.destination_endpoint_key_;
            source_endpoint_key_ = message.source_endpoint_key_;
            message_class_id_ = message.message_class_id_;
            message_data_ = message.message_data_;
            return *this;
        }

        ParticipantGenericMessage& operator=(ParticipantGenericMessage&& message)
        {
            message_identity_ = std::move(message.message_identity_);
            related_message_identity_ = std::move(message.related_message_identity_);
            destination_participant_key_ = std::move(message.destination_participant_key_);
            destination_endpoint_key_ = std::move(message.destination_endpoint_key_);
            source_endpoint_key_ = std::move(message.source_endpoint_key_);
            message_class_id_ = std::move(message.message_class_id_);
            message_data_ = std::move(message.message_data_);
            return *this;
        }

        void message_identity(const MessageIdentity& message_identity)
        {
            message_identity_ = message_identity;
        }

        void message_identity(MessageIdentity&& message_identity)
        {
            message_identity_ = std::move(message_identity);
        }

        const MessageIdentity& message_identity() const
        {
            return message_identity_;
        }

        MessageIdentity& message_identity()
        {
            return message_identity_;
        }

        void related_message_identity(const MessageIdentity& related_message_identity)
        {
            related_message_identity_ = related_message_identity;
        }

        void related_message_identity(MessageIdentity&& related_message_identity)
        {
            related_message_identity_ = std::move(related_message_identity);
        }

        const MessageIdentity& related_message_identity() const
        {
            return related_message_identity_;
        }

        MessageIdentity& related_message_identity()
        {
            return related_message_identity_;
        }

        void destination_participant_key(const GUID_t& destination_participant_key)
        {
            destination_participant_key_ = destination_participant_key;
        }

        void destination_participant_key(GUID_t&& destination_participant_key)
        {
            destination_participant_key_ = std::move(destination_participant_key);
        }

        const GUID_t& destination_participant_key() const
        {
            return destination_participant_key_;
        }

        GUID_t& destination_participant_key()
        {
            return destination_participant_key_;
        }

        void destination_endpoint_key(const GUID_t& destination_endpoint_key)
        {
            destination_endpoint_key_ = destination_endpoint_key;
        }

        void destination_endpoint_key(GUID_t&& destination_endpoint_key)
        {
            destination_endpoint_key_ = std::move(destination_endpoint_key);
        }

        const GUID_t& destination_endpoint_key() const
        {
            return destination_endpoint_key_;
        }

        GUID_t& destination_endpoint_key()
        {
            return destination_endpoint_key_;
        }

        void source_endpoint_key(const GUID_t& source_endpoint_key)
        {
            source_endpoint_key_ = source_endpoint_key;
        }

        void source_endpoint_key(GUID_t&& source_endpoint_key)
        {
            source_endpoint_key_ = std::move(source_endpoint_key);
        }

        const GUID_t& source_endpoint_key() const
        {
            return source_endpoint_key_;
        }

        GUID_t& source_endpoint_key()
        {
            return source_endpoint_key_;
        }

        void message_class_id(const std::string& message_class_id)
        {
            message_class_id_ = message_class_id;
        }

        void message_class_id(std::string&& message_class_id)
        {
            message_class_id_ = std::move(message_class_id);
        }

        const std::string& message_class_id() const
        {
            return message_class_id_;
        }

        std::string& message_class_id()
        {
            return message_class_id_;
        }

        void message_data(const DataHolderSeq& message_data)
        {
            message_data_ = message_data;
        }

        void message_data(DataHolderSeq&& message_data)
        {
            message_data_ = std::move(message_data);
        }

        const DataHolderSeq& message_data() const
        {
            return message_data_;
        }

        DataHolderSeq& message_data()
        {
            return message_data_;
        }

    private:

        MessageIdentity message_identity_;
        MessageIdentity related_message_identity_;
        GUID_t destination_participant_key_;
        GUID_t destination_endpoint_key_;
        GUID_t source_endpoint_key_;
        std::string message_class_id_;
        DataHolderSeq message_data_;
};

class ParticipantGenericMessageHelper
{
    public:

        static size_t serialized_size(const ParticipantGenericMessage& message, size_t current_alignment = 0)
        {
            size_t initial_alignment = current_alignment;

            current_alignment += MessageIdentityHelper::serialized_size(message.message_identity(), current_alignment);
            current_alignment += MessageIdentityHelper::serialized_size(message.related_message_identity(), current_alignment);
            current_alignment += 16 * 3;
            current_alignment += 4 + alignment(current_alignment, 4) + message.message_class_id().size() + 1;
            current_alignment += DataHolderHelper::serialized_size(message.message_data(), current_alignment);

            return current_alignment - initial_alignment;
        }

    private:

        inline static size_t alignment(size_t current_alignment, size_t dataSize) { return (dataSize - (current_alignment % dataSize)) & (dataSize-1);}
};

} //namespace security
} //namespace rtps
} //namespace fastrtps
} //namespace eprosima

#endif // _RTPS_SECURITY_COMMON_PARTICIPANTGENERICMESSAGE_H_
