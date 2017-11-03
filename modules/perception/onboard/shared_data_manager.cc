#include "modules/perception/onboard/shared_data_manager.h"

#include <string>

#include "modules/common/log.h"
#include "modules/perception/onboard/shared_data.h"

namespace apollo {
namespace perception {

bool SharedDataManager::init(const DAGConfig::SharedDataConfig& data_config) {
  if (_inited) {
    AWARN << "SharedDataManager init twice.";
    return true;
  }

  for (auto& proto : data_config.datas()) {
    SharedData* shared_data =
        SharedDataRegisterer::GetInstanceByName(proto.name());

    if (shared_data == NULL) {
      AERROR << "failed to get SharedData instance: " << proto.name();
      return false;
    }
    if (!shared_data->init()) {
      AERROR << "failed to init SharedData. name: " << proto.name();
      return false;
    }

    auto result = _shared_data_map.emplace(
        shared_data->name(), std::unique_ptr<SharedData>(shared_data));

    if (!result.second) {
      AERROR << "duplicate SharedData: " << shared_data->name();
      return true;
    }
  }

  AINFO << "SharedDataManager load " << _shared_data_map.size()
        << " SharedData.";

  _inited = true;
  return true;
}

SharedData* SharedDataManager::get_shared_data(const std::string& name) const {
  auto citer = _shared_data_map.find(name);
  if (citer == _shared_data_map.end()) {
    return NULL;
  }
  return citer->second.get();
}

void SharedDataManager::reset() {
  for (auto& pair : _shared_data_map) {
    pair.second->reset();
  }
  AINFO << "reset all SharedData. nums: " << _shared_data_map.size();
}

void SharedDataManager::remove_stale_data() {
  for (auto& shared_data : _shared_data_map) {
    shared_data.second->remove_stale_data();
  }
}

}  // namespace perception
}  // namespace apollo
