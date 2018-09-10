#include <thread>

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/time/rate.h"
#include "cybertron/time/time.h"
#include "cybertron/transport/message/message_info.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/transport/transport.h"

using apollo::cybertron::Rate;
using apollo::cybertron::Time;
using apollo::cybertron::proto::Chatter;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::transport::MessageInfo;
using apollo::cybertron::transport::QosProfileConf;
using apollo::cybertron::transport::Transport;

uint64_t recv_num = 0;

void MessageCallback(const std::shared_ptr<Chatter>& msg) {
  recv_num++;
  AINFO << "recv_num: " << recv_num;
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);

  bool shutdown = false;
  auto TalkerThreadFunc = [&]() {
    // create talker node
    std::shared_ptr<apollo::cybertron::Node> talker_node(
        apollo::cybertron::CreateNode("talker"));
    if (talker_node == nullptr) {
      AINFO << "create talker node failed.";
      return;
    }

    // fill in talker attributes
    RoleAttributes attr;
    attr.set_channel_name("channel/chatter");
    auto qos = attr.mutable_qos_profile();
    qos->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);

    // create talker
    auto talker = talker_node->CreateWriter<Chatter>(attr);
    if (talker == nullptr) {
      AINFO << "create talker failed.";
    }

    Rate rate(1.0);

    uint64_t seq = 0;
    while (!shutdown) {
      auto msg = std::make_shared<Chatter>();
      msg->set_timestamp(Time::Now().ToNanosecond());
      msg->set_lidar_timestamp(Time::Now().ToNanosecond());
      msg->set_seq(seq);
      msg->set_content("Hello, apollo!");
      talker->Write(msg);
      ++seq;
      AINFO << "send_num: " << seq;
      rate.Sleep();
    }
  };
  // we start talker_thread firstly
  std::thread talker_thread(TalkerThreadFunc);

  // no listener until now
  sleep(5);

  // create listener node
  std::shared_ptr<apollo::cybertron::Node> listener_node(
      apollo::cybertron::CreateNode("listener"));

  if (listener_node == nullptr) {
    AINFO << "create listener node failed.";
    return -1;
  }

  // fill in listener attributes
  RoleAttributes attr;
  attr.set_channel_name("channel/chatter");
  auto qos = attr.mutable_qos_profile();
  qos->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);

  // create listener
  auto listener = listener_node->CreateReader<Chatter>(attr, MessageCallback);

  if (listener == nullptr) {
    std::cout << "create listener failed." << std::endl;
    return -1;
  }

  apollo::cybertron::WaitForShutdown();
  shutdown = true;
  talker_thread.join();
  return 0;
}
