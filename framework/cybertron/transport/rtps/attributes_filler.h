/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_TRANSPORT_RTPS_ATTRIBUTES_FILLER_H_
#define CYBERTRON_TRANSPORT_RTPS_ATTRIBUTES_FILLER_H_

#include <string>

#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include "cybertron/proto/qos_profile.pb.h"

namespace apollo {
namespace cybertron {
namespace transport {

using proto::QosDurabilityPolicy;
using proto::QosHistoryPolicy;
using proto::QosProfile;
using proto::QosReliabilityPolicy;

class AttributesFiller {
 public:
  AttributesFiller();
  virtual ~AttributesFiller();

  static bool FillInPubAttr(const std::string& channel_name,
                            const QosProfile& qos,
                            eprosima::fastrtps::PublisherAttributes* pub_attr);

  static bool FillInSubAttr(const std::string& channel_name,
                            const QosProfile& qos,
                            eprosima::fastrtps::SubscriberAttributes* sub_attr);
};

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_RTPS_ATTRIBUTES_FILLER_H_
