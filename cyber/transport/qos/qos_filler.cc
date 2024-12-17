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

#include "cyber/transport/qos/qos_filler.h"

#include "cyber/common/log.h"
#include "cyber/transport/qos/qos_profile_conf.h"

#include "fastrtps/attributes/PublisherAttributes.h"
#include "fastrtps/attributes/SubscriberAttributes.h"

namespace apollo {
namespace cyber {
namespace transport {

using proto::QosDurabilityPolicy;
using proto::QosHistoryPolicy;
using proto::QosProfile;
using proto::QosReliabilityPolicy;
using transport::QosProfileConf;

bool GetDefaultPubAttributes(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastrtps::PublisherAttributes* pub_attr) {
  RETURN_VAL_IF_NULL2(pub_attr, false);
  pub_attr->topic.topicName = channel_name;
  pub_attr->topic.topicDataType = "UnderlayMessage";
  pub_attr->topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
  switch (qos.history()) {
    case QosHistoryPolicy::HISTORY_KEEP_LAST:
      pub_attr->topic.historyQos.kind =
          eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      break;
    case QosHistoryPolicy::HISTORY_KEEP_ALL:
      pub_attr->topic.historyQos.kind =
          eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      break;
    default:
      break;
  }
  switch (qos.durability()) {
    case QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL:
      pub_attr->qos.m_durability.kind =
          eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case QosDurabilityPolicy::DURABILITY_VOLATILE:
      pub_attr->qos.m_durability.kind =
          eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      break;
    default:
      break;
  }

  switch (qos.reliability()) {
    case QosReliabilityPolicy::RELIABILITY_BEST_EFFORT:
      pub_attr->qos.m_reliability.kind =
          eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      break;
    case QosReliabilityPolicy::RELIABILITY_RELIABLE:
      pub_attr->qos.m_reliability.kind =
          eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      break;
    default:
      break;
  }
  if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
    pub_attr->topic.historyQos.depth = static_cast<int32_t>(qos.depth());
  }

  // ensure the history depth is at least the requested queue size
  RETURN_VAL_IF2(pub_attr->topic.historyQos.depth < 0, false);

  // tranform messages per second to rtps heartbeat
  // set default heartbeat period
  pub_attr->times.heartbeatPeriod.seconds = 0;
  pub_attr->times.heartbeatPeriod.fraction(300 * 4294967);  // 300ms
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
    pub_attr->times.heartbeatPeriod.fraction(fraction);
  }

  pub_attr->qos.m_publishMode.kind =
      eprosima::fastdds::dds::ASYNCHRONOUS_PUBLISH_MODE;
  pub_attr->historyMemoryPolicy =
      eprosima::fastrtps::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
  AINFO << channel_name << "qos: [history: " << pub_attr->topic.historyQos.kind
        << "] [durability: " << pub_attr->qos.m_durability.kind
        << "] [reliability: " << pub_attr->qos.m_reliability.kind
        << "] [depth: " << pub_attr->topic.historyQos.depth
        << "] [samples: " << pub_attr->topic.resourceLimitsQos.max_samples
        << "]";

  return true;
}

bool GetDefaultSubAttributes(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastrtps::SubscriberAttributes* sub_attr) {
  RETURN_VAL_IF_NULL2(sub_attr, false);
  sub_attr->topic.topicName = channel_name;
  sub_attr->topic.topicDataType = "UnderlayMessage";
  sub_attr->topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;

  switch (qos.history()) {
    case QosHistoryPolicy::HISTORY_KEEP_LAST:
      sub_attr->topic.historyQos.kind =
          eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      break;
    case QosHistoryPolicy::HISTORY_KEEP_ALL:
      sub_attr->topic.historyQos.kind =
          eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      break;
    default:
      break;
  }
  switch (qos.durability()) {
    case QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL:
      sub_attr->qos.m_durability.kind =
          eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case QosDurabilityPolicy::DURABILITY_VOLATILE:
      sub_attr->qos.m_durability.kind =
          eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      break;
    default:
      break;
  }

  switch (qos.reliability()) {
    case QosReliabilityPolicy::RELIABILITY_BEST_EFFORT:
      sub_attr->qos.m_reliability.kind =
          eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      break;
    case QosReliabilityPolicy::RELIABILITY_RELIABLE:
      sub_attr->qos.m_reliability.kind =
          eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      break;
    default:
      break;
  }
  if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
    sub_attr->topic.historyQos.depth = static_cast<int32_t>(qos.depth());
  }

  // ensure the history depth is at least the requested queue size
  RETURN_VAL_IF2(sub_attr->topic.historyQos.depth < 0, false);

  sub_attr->historyMemoryPolicy =
      eprosima::fastrtps::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
  AINFO << channel_name << "qos: [history: " << sub_attr->topic.historyQos.kind
        << "] [durability: " << sub_attr->qos.m_durability.kind
        << "] [reliability: " << sub_attr->qos.m_reliability.kind
        << "] [depth: " << sub_attr->topic.historyQos.depth
        << "] [samples: " << sub_attr->topic.resourceLimitsQos.max_samples
        << "]";

  return true;
}

bool GetDefaultTopicAttributes(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastrtps::TopicAttributes* topic_attr) {
  RETURN_VAL_IF_NULL2(topic_attr, false);
  topic_attr->topicName = channel_name;
  topic_attr->topicDataType = "UnderlayMessage";
  topic_attr->topicKind = eprosima::fastrtps::rtps::NO_KEY;

  switch (qos.history()) {
    case QosHistoryPolicy::HISTORY_KEEP_LAST:
      topic_attr->historyQos.kind =
          eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      break;
    case QosHistoryPolicy::HISTORY_KEEP_ALL:
      topic_attr->historyQos.kind =
          eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      break;
    default:
      break;
  }

  if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
    topic_attr->historyQos.depth = static_cast<int32_t>(qos.depth());
  }

  // ensure the history depth is at least the requested queue size
  RETURN_VAL_IF2(topic_attr->historyQos.depth < 0, false);

  return true;
}

///////////////////////////////////////////////////////////////////////////
bool QosFiller::FillInPubQos(const std::string& channel_name,
                             const QosProfile& qos,
                             eprosima::fastdds::dds::PublisherQos* pub_qos) {
  RETURN_VAL_IF_NULL2(pub_qos, false);
  eprosima::fastrtps::PublisherAttributes pub_attr;
  if (!GetDefaultPubAttributes(channel_name, qos, &pub_attr)) {
    return false;
  }
  pub_qos->group_data().setValue(pub_attr.qos.m_groupData);
  pub_qos->partition() = pub_attr.qos.m_partition;
  pub_qos->presentation() = pub_attr.qos.m_presentation;
  return true;
}

bool QosFiller::FillInWriterQos(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastdds::dds::DataWriterQos* writer_qos) {
  RETURN_VAL_IF_NULL2(writer_qos, false);
  eprosima::fastrtps::PublisherAttributes pub_attr;
  if (!GetDefaultPubAttributes(channel_name, qos, &pub_attr)) {
    return false;
  }
  writer_qos->writer_resource_limits().matched_subscriber_allocation =
      pub_attr.matched_subscriber_allocation;
  writer_qos->properties() = pub_attr.properties;
  writer_qos->throughput_controller() = pub_attr.throughputController;
  writer_qos->endpoint().unicast_locator_list = pub_attr.unicastLocatorList;
  writer_qos->endpoint().multicast_locator_list = pub_attr.multicastLocatorList;
  writer_qos->endpoint().remote_locator_list = pub_attr.remoteLocatorList;
  writer_qos->endpoint().history_memory_policy = pub_attr.historyMemoryPolicy;
  writer_qos->endpoint().user_defined_id = pub_attr.getUserDefinedID();
  writer_qos->endpoint().entity_id = pub_attr.getEntityID();
  writer_qos->reliable_writer_qos().times = pub_attr.times;
  writer_qos->reliable_writer_qos().disable_positive_acks =
      pub_attr.qos.m_disablePositiveACKs;
  writer_qos->durability() = pub_attr.qos.m_durability;
  writer_qos->durability_service() = pub_attr.qos.m_durabilityService;
  writer_qos->deadline() = pub_attr.qos.m_deadline;
  writer_qos->latency_budget() = pub_attr.qos.m_latencyBudget;
  writer_qos->liveliness() = pub_attr.qos.m_liveliness;
  writer_qos->reliability() = pub_attr.qos.m_reliability;
  writer_qos->lifespan() = pub_attr.qos.m_lifespan;
  writer_qos->user_data().setValue(pub_attr.qos.m_userData);
  writer_qos->ownership() = pub_attr.qos.m_ownership;
  writer_qos->ownership_strength() = pub_attr.qos.m_ownershipStrength;
  writer_qos->destination_order() = pub_attr.qos.m_destinationOrder;
  writer_qos->representation() = pub_attr.qos.representation;
  writer_qos->publish_mode() = pub_attr.qos.m_publishMode;
  writer_qos->history() = pub_attr.topic.historyQos;
  writer_qos->resource_limits() = pub_attr.topic.resourceLimitsQos;
  return true;
}

bool QosFiller::FillInSubQos(const std::string& channel_name,
                             const QosProfile& qos,
                             eprosima::fastdds::dds::SubscriberQos* sub_qos) {
  RETURN_VAL_IF_NULL2(sub_qos, false);
  eprosima::fastrtps::SubscriberAttributes sub_attr;
  if (!GetDefaultSubAttributes(channel_name, qos, &sub_attr)) {
    return false;
  }
  sub_qos->group_data().setValue(sub_attr.qos.m_groupData);
  sub_qos->partition() = sub_attr.qos.m_partition;
  sub_qos->presentation() = sub_attr.qos.m_presentation;
  return true;
}

bool QosFiller::FillInReaderQos(
    const std::string& channel_name, const proto::QosProfile& qos,
    eprosima::fastdds::dds::DataReaderQos* reader_qos) {
  RETURN_VAL_IF_NULL2(reader_qos, false);
  eprosima::fastrtps::SubscriberAttributes sub_attr;
  if (!GetDefaultSubAttributes(channel_name, qos, &sub_attr)) {
    return false;
  }
  reader_qos->reader_resource_limits().matched_publisher_allocation =
      sub_attr.matched_publisher_allocation;
  reader_qos->properties() = sub_attr.properties;
  reader_qos->expects_inline_qos(sub_attr.expectsInlineQos);
  reader_qos->endpoint().unicast_locator_list = sub_attr.unicastLocatorList;
  reader_qos->endpoint().multicast_locator_list = sub_attr.multicastLocatorList;
  reader_qos->endpoint().remote_locator_list = sub_attr.remoteLocatorList;
  reader_qos->endpoint().history_memory_policy = sub_attr.historyMemoryPolicy;
  reader_qos->endpoint().user_defined_id = sub_attr.getUserDefinedID();
  reader_qos->endpoint().entity_id = sub_attr.getEntityID();
  reader_qos->reliable_reader_qos().times = sub_attr.times;
  reader_qos->reliable_reader_qos().disable_positive_ACKs =
      sub_attr.qos.m_disablePositiveACKs;
  reader_qos->durability() = sub_attr.qos.m_durability;
  reader_qos->durability_service() = sub_attr.qos.m_durabilityService;
  reader_qos->deadline() = sub_attr.qos.m_deadline;
  reader_qos->latency_budget() = sub_attr.qos.m_latencyBudget;
  reader_qos->liveliness() = sub_attr.qos.m_liveliness;
  reader_qos->reliability() = sub_attr.qos.m_reliability;
  reader_qos->lifespan() = sub_attr.qos.m_lifespan;
  reader_qos->user_data().setValue(sub_attr.qos.m_userData);
  reader_qos->ownership() = sub_attr.qos.m_ownership;
  reader_qos->destination_order() = sub_attr.qos.m_destinationOrder;
  reader_qos->type_consistency().type_consistency =
      sub_attr.qos.type_consistency;
  reader_qos->type_consistency().representation = sub_attr.qos.representation;
  reader_qos->time_based_filter() = sub_attr.qos.m_timeBasedFilter;
  reader_qos->history() = sub_attr.topic.historyQos;
  reader_qos->resource_limits() = sub_attr.topic.resourceLimitsQos;
  return true;
}

bool QosFiller::FillInTopicQos(const std::string& channel_name,
                               const proto::QosProfile& qos,
                               eprosima::fastdds::dds::TopicQos* topic_qos) {
  RETURN_VAL_IF_NULL2(topic_qos, false);
  eprosima::fastrtps::TopicAttributes topic_attr;
  if (!GetDefaultTopicAttributes(channel_name, qos, &topic_attr)) {
    return false;
  }
  topic_qos->history() = topic_attr.historyQos;
  topic_qos->resource_limits() = topic_attr.resourceLimitsQos;
  return true;
}
}  // namespace transport
}  // namespace cyber
}  // namespace apollo
