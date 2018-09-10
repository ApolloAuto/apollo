#include "cybertron/cybertron.h"
#include "sensor/sensor_manager.h"

int main(int argc, char *argv[]) {
  apollo::cybertron::Init(argv[0]);
  apollo::sensor::proto::SensorConfig t;
  auto info = t.add_sensors();
  info->set_type(apollo::sensor::proto::Velodyne64);
  info->set_conf_path("conf/velodyne64.conf");

  std::shared_ptr<apollo::cybertron::Node> node(
      apollo::cybertron::CreateNode("sensor_test"));
  auto sm = std::make_shared<apollo::sensor::SensorManager>(t, node);

  sm->Init();
  sm->Start();

  apollo::cybertron::WaitForShutdown();

  return 0;
}
