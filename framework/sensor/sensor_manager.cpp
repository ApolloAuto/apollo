
#include "cybertron/common/environment.h"
#include "cybertron/common/file.h"
#include "cybertron/common/log.h"

#include "sensor/sensor_manager.h"
#include "sensor/velodyne/velodyne_driver.h"

namespace apollo {
namespace sensor {

SensorManager::SensorManager(const proto::SensorConfig& sensor_config,
    const std::shared_ptr<cybertron::Node>& node)
  : sensor_config_(sensor_config)
  , node_(node) {
}

bool SensorManager::Init() {

  epoll_dispatcher_ = EpollDispatcher::Instance();
  for(auto& conf : sensor_config_.sensors()) {
    if (!CreateDriver(conf)) {
      return false;
    }
  }
  return true;
}

bool SensorManager::Start() {

  epoll_dispatcher_->Start();
  epoll_dispatcher_->Join();

}

//TODO:use registy patern for different driver, then create driver by reflection
bool SensorManager::CreateDriver(const proto::SensorInfo& sensor_info) {

  switch(sensor_info.type()) {
    //lidar64 has high packet rate, we send scan for lidar64.
    case proto::Velodyne64:
      {
        proto::VelodyneConfig lidar64_config;
        if (cybertron::common::GetProtoFromFile(
            GetConfPath(sensor_info.conf_path()), &lidar64_config)) {
          auto velo64 = std::make_shared<velodyne::VelodyneDriver>(lidar64_config, node_);
          velo64->Init(epoll_dispatcher_);
          sensor_vector_.push_back(velo64);
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
        return false;
      break;
  }
  return true;
}

std::string SensorManager::GetConfPath(const std::string& conf_path) {
  std::string work_root = cybertron::common::WorkRoot();
  std::string module_root = cybertron::common::ModuleRoot();

  std::string load_path = "";
  if (conf_path[0] == '/') {
    load_path = conf_path;
  } else {
    load_path =
        cybertron::common::GetAbsolutePath(module_root, conf_path);
    if (!cybertron::common::PathExists(load_path)) {
      load_path =
          cybertron::common::GetAbsolutePath(work_root, conf_path);
    }
  }

  if (!cybertron::common::PathExists(load_path)) {
    AERROR << "Path not exist: " << load_path;
  }

  return load_path;
}

} // namespace sensor
} // namespace apollo

