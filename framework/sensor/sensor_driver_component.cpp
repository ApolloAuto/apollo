
#include "sensor/sensor_driver_component.h"

#include "cybertron/common/log.h"
#include "sensor/proto/sensor_config.pb.h"

namespace apollo {
namespace sensor {

SensorDriverComponent::SensorDriverComponent() {
}

bool SensorDriverComponent::Init() {
  apollo::sensor::proto::SensorConfig sensor_config;
  if(!GetProtoConfig(&sensor_config)){
      return false;
    }

  sensor_manager_.reset(new SensorManager(sensor_config, node_));

  sensor_manager_->Init();
  sensor_manager_->Start();
  return true;
}

bool SensorDriverComponent::Proc(const std::shared_ptr<proto::VelodyneScan>& msg) {
  return true;
}

}
}
