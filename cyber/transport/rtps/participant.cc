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
#include "cyber/transport/rtps/participant.h"

#include <vector>

namespace apollo {
namespace cyber {
namespace transport {

auto Participant::CreateSubscriber(const std::string& channel_name,
                                   const proto::QosProfile& qos,
                                   const rtps::subsciber_callback& callback)
    -> std::shared_ptr<transport::Subscriber> {
  if (!participant_) {
    AWARN << "DDSParticipant already released when the subscriber created, "
             "channel:"
          << channel_name;
    return nullptr;
  }
  auto subscriber_ptr = std::make_shared<transport::Subscriber>(
      channel_name, qos, participant_, callback);
  RETURN_VAL_IF(!subscriber_ptr->Init(), nullptr);
  std::lock_guard<std::mutex> lock(subscriber_mutex_);
  subscriber_map_.emplace(channel_name, subscriber_ptr);
  return subscriber_ptr;
}

Participant::Participant(
    const std::string& name, int send_port,
    eprosima::fastdds::dds::DomainParticipantListener* listener)
    : shutdown_(false),
      name_(name),
      send_port_(send_port),
      listener_(listener),
      type_support_(new UnderlayMessageType()),
      participant_(nullptr) {}

Participant::~Participant() {}

bool Participant::Init() {
  return CreateParticipant(name_, send_port_, listener_);
}

void Participant::Shutdown() {
  if (shutdown_.exchange(true)) {
    return;
  }

  publisher_map_.clear();
  subscriber_map_.clear();
  if (listener_ != nullptr) {
    delete listener_;
    listener_ = nullptr;
  }
}

auto Participant::CreatePublisher(const std::string& channel_name,
                                  const proto::QosProfile& qos)
    -> std::shared_ptr<transport::Publisher> {
  if (!participant_) {
    AWARN << "DDSParticipant already released when the publisher created, "
             "channel:"
          << channel_name;
    return nullptr;
  }
  auto publisher_ptr =
      std::make_shared<transport::Publisher>(channel_name, qos, participant_);

  RETURN_VAL_IF(!publisher_ptr->Init(), nullptr);
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  publisher_map_.emplace(channel_name, publisher_ptr);
  return publisher_ptr;
}

bool Participant::CreateParticipant(
    const std::string& name, int send_port,
    eprosima::fastdds::dds::DomainParticipantListener* listener) {
  uint32_t domain_id = 80;
  const char* val = ::getenv("CYBER_DOMAIN_ID");
  if (val != nullptr) {
    try {
      domain_id = std::stoi(val);
    } catch (const std::exception& e) {
      AERROR << "convert domain_id error " << e.what();
      return false;
    }
  }

  std::string ip_env("127.0.0.1");
  const char* ip_val = ::getenv("CYBER_IP");
  if (ip_val != nullptr) {
    ip_env = ip_val;
    if (ip_env.empty()) {
      AERROR << "invalid CYBER_IP (an empty string)";
      return false;
    }
  }

  auto part_attr_conf = std::make_shared<proto::RtpsParticipantAttr>();
  auto& global_conf = common::GlobalData::Instance()->Config();

  if (!global_conf.has_transport_conf() ||
      !global_conf.transport_conf().has_participant_attr()) {
    AERROR << "No rtps participant attr conf.";
    return false;
  }

  part_attr_conf->CopyFrom(global_conf.transport_conf().participant_attr());

  // set wire protocol
  eprosima::fastdds::dds::WireProtocolConfigQos wire_protocol;
  wire_protocol.port.domainIDGain =
      static_cast<uint16_t>(part_attr_conf->domain_id_gain());
  wire_protocol.port.portBase =
      static_cast<uint16_t>(part_attr_conf->port_base());
  wire_protocol.builtin.discovery_config.discoveryProtocol =
      eprosima::fastrtps::rtps::DiscoveryProtocol_t::SIMPLE;
  wire_protocol.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol =
      true;
  wire_protocol.builtin.discovery_config.m_simpleEDP
      .use_PublicationReaderANDSubscriptionWriter = true;
  wire_protocol.builtin.discovery_config.m_simpleEDP
      .use_PublicationWriterANDSubscriptionReader = true;
  wire_protocol.builtin.discovery_config.leaseDuration.seconds =
      part_attr_conf->lease_duration();
  wire_protocol.builtin.discovery_config.leaseDuration_announcementperiod
      .seconds = part_attr_conf->announcement_period();
  wire_protocol.builtin.discovery_config.ignoreParticipantFlags = static_cast<
      eprosima::fastrtps::rtps::ParticipantFilteringFlags>(
      eprosima::fastrtps::rtps::ParticipantFilteringFlags::FILTER_SAME_PROCESS);

  // set transport locator
  eprosima::fastrtps::rtps::Locator_t locator;
  locator.port = 0;
  RETURN_VAL_IF(!eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, ip_env),
                false);
  locator.kind = LOCATOR_KIND_UDPv4;
  wire_protocol.default_unicast_locator_list.push_back(locator);
  wire_protocol.builtin.metatrafficUnicastLocatorList.push_back(locator);
  eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, 239, 255, 0, 1);
  wire_protocol.builtin.metatrafficMulticastLocatorList.push_back(locator);

  // set participant qos
  eprosima::fastdds::dds::PropertyPolicyQos properties;
  eprosima::fastdds::dds::DomainParticipantQos participant_qos;
  participant_qos.name(this->name_.c_str());
  participant_qos.wire_protocol(wire_protocol);
  participant_qos.properties(properties);

  // UDP
  auto udp_transport =
      std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
  udp_transport->sendBufferSize = 1024 * 1025 * 10;
  udp_transport->receiveBufferSize = 1024 * 1025 * 10;
  udp_transport->interfaceWhiteList.push_back(ip_env);
  participant_qos.transport().user_transports.push_back(udp_transport);
  participant_qos.transport().use_builtin_transports = false;
  AINFO << "part name: " << participant_qos.name() << ", port: " << send_port_;

  participant_ =
      eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
          ->create_participant(domain_id, participant_qos, listener_,
                               eprosima::fastdds::dds::StatusMask::none());
  RETURN_VAL_IF_NULL(participant_, false);
  if (type_support_.register_type(participant_) != ReturnCode_t::RETCODE_OK) {
    AERROR << "Register type failed!";
    return false;
  }
  return true;
}

bool Participant::CheckIPVaild(std::string ip_env) {
  struct ifaddrs *ifap, *ifa;
  struct sockaddr_in* sa;
  char ip_address[INET_ADDRSTRLEN];
  if (getifaddrs(&ifap) == -1) {
    AERROR << "getifaddrs error";
    return false;
  }
  std::vector<std::string> ip_vec;
  std::stringstream ip_info;
  ip_info << "All ip info:  \n";
  for (ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET) {
      sa = (struct sockaddr_in*)ifa->ifa_addr;
      inet_ntop(AF_INET, &sa->sin_addr, ip_address, sizeof(ip_address));
      ip_info << "    " << ifa->ifa_name << " " << ip_address << "\n";
      ip_vec.push_back(ip_address);
    }
  }
  AINFO << ip_info.str();
  freeifaddrs(ifap);

  for (std::string ip_interface : ip_vec) {
    if (ip_interface == ip_env) {
      AINFO << "Find same the ip interface in host as cyber ip: " << ip_env;
      return true;
    }
  }
  AERROR << "The same ip interface in host as cyber ip was not found: "
         << ip_env;
  return false;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
