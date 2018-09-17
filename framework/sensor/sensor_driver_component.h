
#ifndef SENSOR_SENSOR_DRIVER_COMPONENT_H_
#define SENSOR_SENSOR_DRIVER_COMPONENT_H_

#include "cybertron/node/node.h"
#include "cybertron/component/component.h"

#include "sensor/proto/sensor_velodyne.pb.h"
#include "sensor/sensor_manager.h"

namespace apollo {
namespace sensor {

using apollo::cybertron::ComponentBase;
class SensorDriverComponent : public cybertron::Component<proto::VelodyneScan> {
 public:
  SensorDriverComponent();
  bool Init() override;
  bool Proc(const std::shared_ptr<proto::VelodyneScan>& msg) override;

 private:
  std::unique_ptr<SensorManager> sensor_manager_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(SensorDriverComponent)
}
}

#endif
