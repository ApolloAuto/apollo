#ifndef MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_
#define MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_

#include <map>
#include <memory>
#include <string>

#include "modules/common/macro.h"
#include "modules/perception/onboard/proto/dag_config.pb.h"
#include "modules/perception/onboard/shared_data.h"

namespace apollo {
namespace perception {

class SharedDataManager {
 public:
  SharedDataManager() = default;
  ~SharedDataManager() = default;

  bool init(const DAGConfig::SharedDataConfig& data_config);

  // thread-safe.
  SharedData* get_shared_data(const std::string& name) const;

  void reset();
  void remove_stale_data();

 private:
  DISALLOW_COPY_AND_ASSIGN(SharedDataManager);

  using SharedDataMap = std::map<std::string, std::unique_ptr<SharedData>>;

  SharedDataMap _shared_data_map;
  bool _inited = false;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_
