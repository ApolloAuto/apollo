/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/transport/rtps/attributes_filler.h"

#include <limits>

#include "cyber/common/log.h"
#include "cyber/transport/qos/qos_profile_conf.h"

namespace apollo {
namespace cyber {
namespace transport {

AttributesFiller::AttributesFiller() {}
AttributesFiller::~AttributesFiller() {}

bool AttributesFiller::FillInPubAttr(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastrtps::PublisherAttributes* pub_attr) {
  RETURN_VAL_IF_NULL(pub_attr, false);

  pub_attr->topic.topicName = channel_name;
  pub_attr->topic.topicDataType = "UnderlayMessage";
  pub_attr->topic.topicKind = eprosima::fastrtps::NO_KEY;

  switch (qos.history()) {
    case QosHistoryPolicy::HISTORY_KEEP_LAST:
      pub_attr->topic.historyQos.kind =
          eprosima::fastrtps::KEEP_LAST_HISTORY_QOS;
      break;
    case QosHistoryPolicy::HISTORY_KEEP_ALL:
      pub_attr->topic.historyQos.kind =
          eprosima::fastrtps::KEEP_ALL_HISTORY_QOS;
      break;
    default:
      break;
  }

  switch (qos.durability()) {
    case QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL:
      pub_attr->qos.m_durability.kind =
          eprosima::fastrtps::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case QosDurabilityPolicy::DURABILITY_VOLATILE:
      pub_attr->qos.m_durability.kind =
          eprosima::fastrtps::VOLATILE_DURABILITY_QOS;
      break;
    default:
      break;
  }

  switch (qos.reliability()) {
    case QosReliabilityPolicy::RELIABILITY_BEST_EFFORT:
      pub_attr->qos.m_reliability.kind =
          eprosima::fastrtps::BEST_EFFORT_RELIABILITY_QOS;
      break;
    case QosReliabilityPolicy::RELIABILITY_RELIABLE:
      pub_attr->qos.m_reliability.kind =
          eprosima::fastrtps::RELIABLE_RELIABILITY_QOS;
      break;
    default:
      break;
  }

  if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
    pub_attr->topic.historyQos.depth = static_cast<int32_t>(qos.depth());
  }

  // ensure the history depth is at least the requested queue size
  if (pub_attr->topic.historyQos.depth < 0) {
    return false;
  }

  // transform messages per second to rtps heartbeat
  // set default heartbeat period
  pub_attr->times.heartbeatPeriod.seconds = 1;
  pub_attr->times.heartbeatPeriod.fraction = 0;
  if (qos.mps() != 0) {
    uint64_t mps = qos.mps();

    // adapt heartbeat period
    if (mps > 1024) {
      mps = 1024;
    } else if (mps < 64) {
      mps = 64;
    }

    uint64_t fractions = (256ull << 32) / mps;
    uint32_t fraction = fractions & 0xffffffff;
    int32_t seconds = static_cast<int32_t>(fractions >> 32);

    pub_attr->times.heartbeatPeriod.seconds = seconds;
    pub_attr->times.heartbeatPeriod.fraction = fraction;
  }

  pub_attr->qos.m_publishMode.kind =
      eprosima::fastrtps::ASYNCHRONOUS_PUBLISH_MODE;
  pub_attr->historyMemoryPolicy =
      eprosima::fastrtps::DYNAMIC_RESERVE_MEMORY_MODE;
  pub_attr->topic.resourceLimitsQos.max_samples = 10000;

  return true;
}

bool AttributesFiller::FillInSubAttr(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastrtps::SubscriberAttributes* sub_attr) {
  RETURN_VAL_IF_NULL(sub_attr, false);
  sub_attr->topic.topicName = channel_name;
  sub_attr->topic.topicDataType = "UnderlayMessage";
  sub_attr->topic.topicKind = eprosima::fastrtps::NO_KEY;

  switch (qos.history()) {
    case QosHistoryPolicy::HISTORY_KEEP_LAST:
      sub_attr->topic.historyQos.kind =
          eprosima::fastrtps::KEEP_LAST_HISTORY_QOS;
      break;
    case QosHistoryPolicy::HISTORY_KEEP_ALL:
      sub_attr->topic.historyQos.kind =
          eprosima::fastrtps::KEEP_ALL_HISTORY_QOS;
      break;
    default:
      break;
  }

  switch (qos.durability()) {
    case QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL:
      sub_attr->qos.m_durability.kind =
          eprosima::fastrtps::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case QosDurabilityPolicy::DURABILITY_VOLATILE:
      sub_attr->qos.m_durability.kind =
          eprosima::fastrtps::VOLATILE_DURABILITY_QOS;
      break;
    default:
      break;
  }

  switch (qos.reliability()) {
    case QosReliabilityPolicy::RELIABILITY_BEST_EFFORT:
      sub_attr->qos.m_reliability.kind =
          eprosima::fastrtps::BEST_EFFORT_RELIABILITY_QOS;
      break;
    case QosReliabilityPolicy::RELIABILITY_RELIABLE:
      sub_attr->qos.m_reliability.kind =
          eprosima::fastrtps::RELIABLE_RELIABILITY_QOS;
      break;
    default:
      break;
  }

  if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
    sub_attr->topic.historyQos.depth = static_cast<int32_t>(qos.depth());
  }

  // ensure the history depth is at least the requested queue size
  if (sub_attr->topic.historyQos.depth < 0) {
    return false;
  }

  sub_attr->historyMemoryPolicy =
      eprosima::fastrtps::DYNAMIC_RESERVE_MEMORY_MODE;
  sub_attr->topic.resourceLimitsQos.max_samples = 10000;

  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
