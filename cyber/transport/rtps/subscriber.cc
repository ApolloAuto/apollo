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
#include "cyber/transport/rtps/subscriber.h"

namespace apollo {
namespace cyber {
namespace transport {

Subscriber::Subscriber(const std::string& name, const proto::QosProfile& qos,
                       eprosima::fastdds::dds::DomainParticipant* participant,
                       const rtps::subsciber_callback& callback)
    : channel_name_(name),
      qos_(qos),
      shutdown_(false),
      callback_(callback),
      subscriber_listener_(nullptr),
      participant_(participant),
      subscriber_(nullptr),
      topic_(nullptr),
      reader_(nullptr) {}

Subscriber::~Subscriber() {}

bool Subscriber::Init() {
  eprosima::fastdds::dds::SubscriberQos sub_qos;
  RETURN_VAL_IF(
      !QosFiller::FillInSubQos(this->channel_name_, this->qos_, &sub_qos),
      false);
  subscriber_ = participant_->create_subscriber(sub_qos);
  if (subscriber_ == nullptr) {
    AINFO << "something went wrong while creating the fastdds subscriber...";
    return false;
  }

  if (!EnsureCreateTopic(this->channel_name_)) {
    AINFO << "something went wrong while creating the fastdds topic...";
    return false;
  }

  eprosima::fastdds::dds::DataReaderQos reader_qos;
  RETURN_VAL_IF(
      !QosFiller::FillInReaderQos(this->channel_name_, this->qos_, &reader_qos),
      false);
  subscriber_listener_ = new dispatcher::SubscriberListener(this->callback_);

  reader_ =
      subscriber_->create_datareader(topic_, reader_qos, subscriber_listener_);
  if (reader_ == nullptr) {
    AINFO << "something went wrong while creating the fastdds datareader...";
    return false;
  }

  ADEBUG << "dds reader: " << reader_ << ", subscriber: " << this
         << ", reader guid: " << reader_->guid()
         << ", channel: " << this->channel_name_;
  return true;
}

void Subscriber::Shutdown() {
  RETURN_IF(shutdown_.exchange(true));

  if (subscriber_ != nullptr && reader_ != nullptr) {
    subscriber_->delete_datareader(reader_);
    reader_ = nullptr;
  }
  if (participant_ != nullptr && subscriber_ != nullptr) {
    if (participant_->delete_subscriber(subscriber_) ==
        eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
      subscriber_ = nullptr;
    } else {
      AERROR << channel_name_ << ": Failed to delete the subscriber.";
    }
  }
  if (participant_ != nullptr && topic_ != nullptr) {
    participant_->delete_topic(topic_);
    topic_ = nullptr;
  }

  if (subscriber_listener_ != nullptr) {
    delete subscriber_listener_;
  }
}

bool Subscriber::EnsureCreateTopic(const std::string& channel_name) {
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
