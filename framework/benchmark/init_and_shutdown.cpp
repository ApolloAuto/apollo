#include <cstdint>
#include <string>

#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/time/time.h"

using apollo::cybertron::Node;
using apollo::cybertron::Time;
using apollo::cybertron::proto::Chatter;
using apollo::cybertron::topology::Topology;

int main(int argc, char *argv[]) {
  AINFO << "benchmark start.";
  apollo::cybertron::Init(argv[0]);

  std::string timestamp("");

  try {
    timestamp = std::to_string(Time::Now().ToNanosecond());
  } catch (...) {
  }

  // create node
  std::string node_name = "node" + timestamp;
  auto node = apollo::cybertron::CreateNode(node_name);
  std::string channel_name = "channel" + timestamp;
  // create writer/reader
  auto writer = node->CreateWriter<Chatter>(channel_name);
  auto reader = node->CreateReader<Chatter>(
      channel_name, [](const std::shared_ptr<Chatter>& msg) { (void)msg; });

  std::string service_name = "service" + timestamp;
  // create service/client
  try {
    auto service = node->CreateService<Chatter, Chatter>(
        service_name,
        [](const std::shared_ptr<Chatter>& req, std::shared_ptr<Chatter>& rsp) {
          (void)req;
          (void)rsp;
        });
  } catch (...) {
  }

  auto client = node->CreateClient<Chatter, Chatter>(service_name);

  auto topology = Topology::Instance();
  while (apollo::cybertron::OK()) {
    sleep(1);
  }

  AINFO << "benchmark end.";
  return 0;
}
