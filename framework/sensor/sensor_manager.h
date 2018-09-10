
#ifndef SENSOR_SENSOR_MANAGER_H_
#define SENSOR_SENSOR_MANAGER_H_ 

#include "cybertron/node/node.h"
#include "sensor/proto/sensor_config.pb.h"
 
#include "sensor/epoll_dispatcher.h" 
#include "sensor/sensor.h" 

namespace apollo {
namespace sensor {

//Load all into epoll dispatcher 
class SensorManager {
 public:
  SensorManager(const proto::SensorConfig& sensor_config, 
    const std::shared_ptr<cybertron::Node>& node);
  virtual ~SensorManager() {}
  bool Init();
  bool Start();

 private:
  bool CreateDriver(const proto::SensorInfo& sensor_info);
  std::string GetConfPath(const std::string& conf_path);

  proto::SensorConfig sensor_config_;
  std::shared_ptr<EpollDispatcher> epoll_dispatcher_ = nullptr;
  std::shared_ptr<cybertron::Node> node_ = nullptr;
  std::vector<std::shared_ptr<Sensor>> sensor_vector_;

  SensorManager(SensorManager &) = delete;
  SensorManager &operator=(SensorManager &) = delete;
};

} // namespace sensor 
} // namespace apollo 

#endif //SENSOR_EPOLL_HANDLER_H
