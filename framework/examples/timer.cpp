
#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"

using apollo::cybertron::Node;
int main(int argc, char *argv[]) {
  apollo::cybertron::Init(argv[0]);

  std::shared_ptr<Node> node(apollo::cybertron::CreateNode("parameter"));
  uint64_t count = 0;
  auto timer = apollo::cybertron::Timer(1000,
                                        [&count]() {
                                          AINFO << "timer shot count: "
                                                << count;
                                          count++;
                                        },
                                        true);
  timer.Start();
  apollo::cybertron::WaitForShutdown();
  return 0;
}
