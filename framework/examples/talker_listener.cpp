#include <iostream>
#include <thread>

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/proto/common_geometry.pb.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/time/rate.h"
#include "cybertron/time/time.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/topology/topology.h"

using apollo::cybertron::Rate;
using apollo::cybertron::Time;

void MessageCallback(
    const std::shared_ptr<apollo::cybertron::proto::Chatter>& msg) {
  // std::cout << "listener received a new message!" << std::endl;
  std::cout << "111 sequence: " << msg->seq() << std::endl;
  // std::cout << "content: " << msg->content() << std::endl;
}

void MessageCallback2(
    const std::shared_ptr<apollo::cybertron::proto::Chatter>& msg) {
  std::cout << "222 sequence: " << msg->seq() << std::endl;
}

int main(int argc, char *argv[]) {
  apollo::cybertron::Init(argv[0]);

  auto TalkerThreadFunc = []() {
    // create talker node
    std::shared_ptr<apollo::cybertron::Node> talker_node(
        apollo::cybertron::CreateNode("talker"));
    if (talker_node == nullptr) {
      std::cout << "create talker node failed." << std::endl;
      return;
    }

    // fill in talker attributes
    apollo::cybertron::proto::RoleAttributes attr;
    attr.set_channel_name("channel/chatter");
    auto qos = attr.mutable_qos_profile();
    qos->CopyFrom(
        apollo::cybertron::transport::QosProfileConf::QOS_PROFILE_DEFAULT);

    // create talker
    auto talker =
        talker_node->CreateWriter<apollo::cybertron::proto::Chatter>(attr);
    if (talker == nullptr) {
      std::cout << "create talker failed." << std::endl;
    }

    Rate rate(1.0);

    auto msg = std::make_shared<apollo::cybertron::proto::Chatter>();
    uint64_t seq = 0;
    while (apollo::cybertron::OK()) {
      msg->set_timestamp(Time::Now().ToNanosecond());
      msg->set_lidar_timestamp(Time::Now().ToNanosecond());
      msg->set_seq(seq);
      msg->set_content("Hello, apollo!");
      talker->Write(msg);
      std::cout << "talker sent a message!" << std::endl;
      ++seq;
      rate.Sleep();
    }
  };

  // create listener node
  std::shared_ptr<apollo::cybertron::Node> listener_node(
      apollo::cybertron::CreateNode("listener"));

  if (listener_node == nullptr) {
    std::cout << "create listener node failed." << std::endl;
    return -1;
  }

  // fill in listener attributes
  apollo::cybertron::proto::RoleAttributes attr;
  attr.set_channel_name("channel/chatter");
  attr.set_node_name("listener");
  auto qos = attr.mutable_qos_profile();
  qos->CopyFrom(
      apollo::cybertron::transport::QosProfileConf::QOS_PROFILE_DEFAULT);

  // create listener
  auto listener =
      listener_node->CreateReader<apollo::cybertron::proto::Chatter>(
          attr, MessageCallback);

  if (listener == nullptr) {
    std::cout << "create listener failed." << std::endl;
    return -1;
  }

  // create listener2 node
  std::shared_ptr<apollo::cybertron::Node> listener_node2(
      apollo::cybertron::CreateNode("listener2"));

  if (listener_node2 == nullptr) {
    std::cout << "create listener2 node failed." << std::endl;
    return -1;
  }

  // create listener2
  auto listener2 =
      listener_node2->CreateReader<apollo::cybertron::proto::Chatter>(
          attr, MessageCallback2);

  if (listener2 == nullptr) {
    std::cout << "create listener failed." << std::endl;
    return -1;
  }

  std::thread talker_thread(TalkerThreadFunc);
  apollo::cybertron::WaitForShutdown();
  talker_thread.join();

  return 0;
}
