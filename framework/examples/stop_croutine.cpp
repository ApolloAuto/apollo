#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/util.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/cybertron.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/scheduler.h"

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::croutine::CreateRoutineFactory;
using apollo::cybertron::croutine::RoutineFactory;
using apollo::cybertron::data::DataVisitor;
using apollo::cybertron::scheduler::Scheduler;

struct DemoMsg {
  int id = 0;
};

void Proc(const std::shared_ptr<DemoMsg>& msg) {
  AINFO << "Process message, id: " << msg->id;
  while (true) {
    usleep(200000);
  }
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);

  auto sched = Scheduler::Instance();
  sched->RemoveTask("testxxx");
  std::string channel_name = "hello/apollo";
  std::string node_name = "remove_node";
  auto dv = std::make_shared<DataVisitor<DemoMsg>>(
      apollo::cybertron::common::Hash(channel_name), 1);
  auto func = [=](const std::shared_ptr<DemoMsg>& msg) { Proc(msg); };
  auto factory = CreateRoutineFactory<DemoMsg>(func, dv);
  sched->CreateTask(factory, node_name);
  sched->RemoveTask(node_name);
  sleep(1);
  sched->RemoveTask(node_name);
  return 0;
}
