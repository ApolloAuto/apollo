#include <unistd.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "cybertron/cybertron.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/proto/perception.pb.h"

#include "cybertron/class_loader/class_loader_manager.h"
#include "cybertron/scheduler/scheduler.h"

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::transport::MessageInfo;
using apollo::cybertron::Reader;
using apollo::cybertron::Writer;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::proto::ComponentConfig;

apollo::cybertron::class_loader::ClassLoaderManager loader;

class PlanningComponent;

std::string channel_per = "/perception/channel";
std::string channel_dir = "/driver/channel";
std::string perception_lib =
    "/apollo/framework/install/lib/libperception_component.so";

std::shared_ptr<ComponentBase> perception = nullptr;
std::shared_ptr<PlanningComponent> planning = nullptr;
std::shared_ptr<Writer<Driver>> driver_writer = nullptr;
std::shared_ptr<apollo::cybertron::Node> node = nullptr;

/*
 * Planning Component
 */
class PlanningComponent : public Component<Perception> {
 public:
  bool Init() {}
  bool Proc(const std::shared_ptr<Perception>& msg) override;
};

bool PlanningComponent::Proc(const std::shared_ptr<Perception>& msg) {
  AINFO << std::this_thread::get_id() << ": Start " << __FUNCTION__ << "["
        << msg->msg_id() << "]" << std::endl;
  auto out_msg = std::make_shared<Perception>();
  double result = 0.0;
  for (int i = 0; i < 10000000; i++) {
    result += tan(
        atan(tan(atan(tan(atan(tan(atan(tan(atan(123456789.123456789))))))))));
  }
  out_msg->set_result(result);
}

void RunDriver() {
  // Mock Drivers
  RoleAttributes attr;
  attr.set_node_name("Driver");
  attr.set_channel_name(channel_dir);
  driver_writer = node->CreateWriter<Driver>(attr);
  if (driver_writer == nullptr) {
    AINFO << "Driver create writer failed." << std::endl;
    return;
  }

  for (int i = 0; i < 100; ++i) {
    auto msg = std::make_shared<Driver>();
    msg->set_msg_id(i);
    driver_writer->Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void InitPerception() {
  ComponentConfig config;
  config.set_name("Perception");
  config.add_readers()->set_channel(channel_dir);

  if (loader.LoadLibrary(perception_lib)) {
    perception = loader.CreateClassObj<ComponentBase>("PerceptionComponent");
    if (perception != nullptr) {
      perception->Initialize(config);
    } else {
      AINFO << "Can't create" << std::endl;
    }
  } else {
    AERROR << "Initialize perception component failed!";
  }
}

void InitPlanning() {
  ComponentConfig config;
  config.set_name("Planning");
  config.add_readers()->set_channel(channel_per);
  planning.reset(new PlanningComponent());
  planning->Initialize(config);
}

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  node = apollo::cybertron::CreateNode("start_node");
  InitPerception();
  InitPlanning();
  RunDriver();
  //  apollo::cybertron::PrintSchedulerStatistics();
  return 0;
}
