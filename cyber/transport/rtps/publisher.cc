/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/transport/rtps/publisher.h"
#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

Publisher::Publisher(const std::string& channel_name,
                     const proto::QosProfile& qos,
                     eprosima::fastdds::dds::DomainParticipant* participant)
    : channel_name_(channel_name),
      qos_(qos),
      shutdown_(false),
      participant_(participant),
      publisher_(nullptr),
      topic_(nullptr),
      writer_(nullptr) {}

Publisher::~Publisher() { Shutdown(); }

bool Publisher::Init() {
  eprosima::fastdds::dds::PublisherQos pub_qos;
  RETURN_VAL_IF(
      !QosFiller::FillInPubQos(this->channel_name_, this->qos_, &pub_qos),
      false);
  publisher_ = participant_->create_publisher(pub_qos, nullptr);
  if (publisher_ == nullptr) {
    AINFO << "something went wrong while creating the publisher...";
    return false;
  }

  if (!EnsureCreateTopic(this->channel_name_)) {
    AINFO << "something went wrong while creating the topic...";
    return false;
  }

  eprosima::fastdds::dds::DataWriterQos writer_qos;
  RETURN_VAL_IF(
      !QosFiller::FillInWriterQos(this->channel_name_, this->qos_, &writer_qos),
      false);
  writer_ = publisher_->create_datawriter(topic_, writer_qos, nullptr);
  if (writer_ == nullptr) {
    AINFO << "something went wrong while creating the datawriter...";
    return false;
  }

  return true;
}

bool Publisher::Write(const UnderlayMessage& msg, bool is_topo_msg) {
  RETURN_VAL_IF(shutdown_.load(), false);
  if (is_topo_msg) {
    AINFO << "FastDDSPublisher::Write data size: " << msg.data().size();
  }
  return writer_->write(
      reinterpret_cast<void*>(const_cast<UnderlayMessage*>(&msg)));
}

bool Publisher::Write(const UnderlayMessage& msg, const MessageInfo& msg_info,
                      bool is_topo_msg) {
  RETURN_VAL_IF(shutdown_.load(), false);
  if (is_topo_msg) {
    AINFO << "FastDDSPublisher::Write data size: " << msg.data().size();
  }

  eprosima::fastrtps::rtps::WriteParams wparams;

  char* ptr =
      reinterpret_cast<char*>(&wparams.related_sample_identity().writer_guid());

  memcpy(ptr, msg_info.sender_id().data(), ID_SIZE);
  memcpy(ptr + ID_SIZE, msg_info.spare_id().data(), ID_SIZE);

  wparams.related_sample_identity().sequence_number().high =
      (int32_t)((msg_info.seq_num() & 0xFFFFFFFF00000000) >> 32);
  wparams.related_sample_identity().sequence_number().low =
      (int32_t)(msg_info.seq_num() & 0xFFFFFFFF);

  return writer_->write(
      reinterpret_cast<void*>(const_cast<UnderlayMessage*>(&msg)), wparams);
}

void Publisher::Shutdown() {
  RETURN_IF(shutdown_.exchange(true));

  if (publisher_ != nullptr && writer_ != nullptr) {
    publisher_->delete_datawriter(writer_);
    writer_ = nullptr;
  }
  if (participant_ != nullptr && publisher_ != nullptr) {
    if (participant_->delete_publisher(publisher_) ==
        eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
      publisher_ = nullptr;
    } else {
      AERROR << channel_name_ << ": Failed to delete the publisher.";
    }
  }
  if (participant_ != nullptr && topic_ != nullptr) {
    participant_->delete_topic(topic_);
    topic_ = nullptr;
  }
}

bool Publisher::EnsureCreateTopic(const std::string& channel_name) {
  topic_ = dynamic_cast<eprosima::fastdds::dds::Topic*>(
      participant_->lookup_topicdescription(channel_name));
  if (topic_ == nullptr) {
    eprosima::fastdds::dds::TopicQos topic_qos;
    RETURN_VAL_IF(
        !QosFiller::FillInTopicQos(channel_name, this->qos_, &topic_qos),
        false);
    topic_ =
        participant_->create_topic(channel_name, "UnderlayMessage", topic_qos);
  }
  return (topic_ != nullptr);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
