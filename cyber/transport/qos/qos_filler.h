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

#ifndef CYBER_DDS_ADAPTER_FASTDDS_QOS_FILLER_H_
#define CYBER_DDS_ADAPTER_FASTDDS_QOS_FILLER_H_

#include <string>

#include "cyber/base/macros.h"

#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/subscriber/qos/SubscriberQos.hpp"
#include "fastdds/dds/topic/qos/TopicQos.hpp"
#include "cyber/proto/qos_profile.pb.h"

namespace apollo {
namespace cyber {
namespace transport {

class QosFiller {
 public:
  static bool FillInPubQos(const std::string& channel_name,
                           const proto::QosProfile& qos,
                           eprosima::fastdds::dds::PublisherQos* pub_qos);

  static bool FillInWriterQos(
      const std::string& channel_name, const proto::QosProfile& qos,
      eprosima::fastdds::dds::DataWriterQos* writer_qos);

  static bool FillInSubQos(const std::string& channel_name,
                           const proto::QosProfile& qos,
                           eprosima::fastdds::dds::SubscriberQos* sub_qos);

  static bool FillInReaderQos(
      const std::string& channel_name, const proto::QosProfile& qos,
      eprosima::fastdds::dds::DataReaderQos* reader_qos);

  static bool FillInTopicQos(const std::string& channel_name,
                             const proto::QosProfile& qos,
                             eprosima::fastdds::dds::TopicQos* topic_qos);
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DDS_ADAPTER_FASTDDS_QOS_FILLER_H_
