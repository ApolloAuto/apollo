
#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"

using apollo::cybertron::Node;
int main(int argc, char *argv[]) {
  apollo::cybertron::Init(argv[0]);

  std::shared_ptr<Node> node(apollo::cybertron::CreateNode("parameter"));
  uint64_t count = 0;
  auto timer1 = apollo::cybertron::Timer(10,
                                        [&count]() {
                                          AINFO << "timer1 shot count: "
                                                << count;
                                          count++;
                                        },
                                        false);
  auto timer2 = apollo::cybertron::Timer(10,
                                        [&count]() {
                                          AINFO << "timer2 shot count: "
                                                << count;
                                          count++;
                                        },
                                        false);
  timer1.Start();
  timer2.Start();
  sleep(10);
  timer1.Stop();
  timer2.Stop();
  apollo::cybertron::WaitForShutdown();
  return 0;
}
