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

#include "cyber/transport/rtps/participant.h"

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/proto/transport_conf.pb.h"

namespace apollo {
namespace cyber {
namespace transport {

Participant::Participant(const std::string& name, int send_port,
                         eprosima::fastrtps::ParticipantListener* listener)
    : shutdown_(false),
      name_(name),
      send_port_(send_port),
      listener_(listener),
      fastrtps_participant_(nullptr) {}

Participant::~Participant() {}

void Participant::Shutdown() {
  if (shutdown_.exchange(true)) {
    return;
  }

  std::lock_guard<std::mutex> lk(mutex_);
  if (fastrtps_participant_ != nullptr) {
    eprosima::fastrtps::Domain::removeParticipant(fastrtps_participant_);
    fastrtps_participant_ = nullptr;
    listener_ = nullptr;
  }
}

eprosima::fastrtps::Participant* Participant::fastrtps_participant() {
  if (shutdown_.load()) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lk(mutex_);
  if (fastrtps_participant_ != nullptr) {
    return fastrtps_participant_;
  }

  CreateFastRtpsParticipant(name_, send_port_, listener_);
  return fastrtps_participant_;
}

void Participant::CreateFastRtpsParticipant(
    const std::string& name, int send_port,
    eprosima::fastrtps::ParticipantListener* listener) {
  uint32_t domain_id = 80;

  const char* val = ::getenv("CYBER_DOMAIN_ID");
  if (val != nullptr) {
    try {
      domain_id = std::stoi(val);
    } catch (const std::exception& e) {
      AERROR << "convert domain_id error " << e.what();
      return;
    }
  }

  auto part_attr_conf = std::make_shared<proto::RtpsParticipantAttr>();
  auto& global_conf = common::GlobalData::Instance()->Config();
  if (global_conf.has_transport_conf() &&
      global_conf.transport_conf().has_participant_attr()) {
    part_attr_conf->CopyFrom(global_conf.transport_conf().participant_attr());
  }

  eprosima::fastrtps::ParticipantAttributes attr;
  attr.rtps.defaultSendPort = send_port;
  attr.rtps.port.domainIDGain =
      static_cast<uint16_t>(part_attr_conf->domain_id_gain());
  attr.rtps.port.portBase = static_cast<uint16_t>(part_attr_conf->port_base());
  attr.rtps.use_IP6_to_send = false;
  attr.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
  attr.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
  attr.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter =
      true;
  attr.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader =
      true;
  attr.rtps.builtin.domainId = domain_id;

  /**
   * The user should set the lease_duration and the announcement_period with
   * values that differ in at least 30%. Values too close to each other may
   * cause the failure of the writer liveliness assertion in networks with high
   * latency or with lots of communication errors.
   */
  attr.rtps.builtin.leaseDuration.seconds = part_attr_conf->lease_duration();
  attr.rtps.builtin.leaseDuration_announcementperiod.seconds =
      part_attr_conf->announcement_period();

  attr.rtps.setName(name.c_str());

  std::string ip_env("127.0.0.1");
  const char* ip_val = ::getenv("CYBER_IP");
  if (ip_val != nullptr) {
    ip_env = ip_val;
    if (ip_env.empty()) {
      AERROR << "invalid CYBER_IP (an empty string)";
      return;
    }
  }
  ADEBUG << "cyber ip: " << ip_env;

  eprosima::fastrtps::rtps::Locator_t locator;
  locator.port = 0;
  RETURN_IF(!locator.set_IP4_address(ip_env));

  locator.kind = LOCATOR_KIND_UDPv4;

  attr.rtps.defaultUnicastLocatorList.push_back(locator);
  attr.rtps.defaultOutLocatorList.push_back(locator);
  attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(locator);

  locator.set_IP4_address(239, 255, 0, 1);
  attr.rtps.builtin.metatrafficMulticastLocatorList.push_back(locator);

  fastrtps_participant_ =
      eprosima::fastrtps::Domain::createParticipant(attr, listener);
  RETURN_IF_NULL(fastrtps_participant_);
  eprosima::fastrtps::Domain::registerType(fastrtps_participant_, &type_);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
