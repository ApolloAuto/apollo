
#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/driver.pb.h"

using apollo::cybertron::proto::Driver;

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);

  std::shared_ptr<apollo::cybertron::Node> node(
      apollo::cybertron::CreateNode("start_node"));
  auto server = node->CreateService<Driver, Driver>(
      "test_server", [](const std::shared_ptr<Driver>& request,
                        std::shared_ptr<Driver>& response) {
        AINFO << "i am driver server";
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(0);
      });

  auto client = node->CreateClient<Driver, Driver>("test_server");

  auto driver_msg = std::make_shared<Driver>();
  driver_msg->set_msg_id(0);
  driver_msg->set_timestamp(0);

  while (apollo::cybertron::OK()) {
    auto res = client->SendRequest(driver_msg);
    if (res != nullptr) {
      AINFO << "responese:" << res->ShortDebugString();
    } else {
      AINFO << "service may not ready.";
    }

    sleep(1);
  }

  return 0;
}
