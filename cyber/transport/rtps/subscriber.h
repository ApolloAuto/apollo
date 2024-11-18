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
#ifndef CYBER_TRANSPORT_RTPS_SUBSCRIBER_H
#define CYBER_TRANSPORT_RTPS_SUBSCRIBER_H

#include <functional>
#include <memory>
#include <string>

#include "cyber/base/macros.h"

#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"

#include "cyber/proto/qos_profile.pb.h"

#include "cyber/common/log.h"
#include "cyber/service_discovery/communication/subscriber_listener.h"
#include "cyber/transport/dispatcher/subscriber_listener.h"
#include "cyber/transport/message/message_info.h"
#include "cyber/transport/qos/qos_filler.h"
#include "cyber/transport/rtps/underlay_message.h"

namespace apollo {
namespace cyber {
namespace transport {
class Subscriber {
 public:
  Subscriber(const std::string& name, const proto::QosProfile& qos,
             eprosima::fastdds::dds::DomainParticipant* participant,
             const rtps::subsciber_callback& callback);
  virtual ~Subscriber();

  bool Init();
  void Shutdown();

 private:
  Subscriber(const Subscriber&) = delete;
  Subscriber& operator=(const Subscriber&) = delete;
  bool EnsureCreateTopic(const std::string& channel_name);

  std::string channel_name_;
  proto::QosProfile qos_;
  std::atomic<bool> shutdown_;
  rtps::subsciber_callback callback_;

  eprosima::fastdds::dds::SubscriberListener* subscriber_listener_;
  eprosima::fastdds::dds::DomainParticipant* participant_;
  eprosima::fastdds::dds::Subscriber* subscriber_;
  eprosima::fastdds::dds::Topic* topic_;
  eprosima::fastdds::dds::DataReader* reader_;
};
}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_RTPS_SUBSCRIBER_H
