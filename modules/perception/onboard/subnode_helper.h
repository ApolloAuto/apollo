#ifndef MODEULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_
#define MODEULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_

#include <map>
#include <string>

#include "modules/common/macro.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
namespace perception {

class SubnodeHelper {
 public:
  // subnode has a field named reserve
  // when reserve like "source_name:./data;source_param:.fsi"
  // you can use this func to get the map
  static bool ParseReserveField(const std::string &reserve,
                                std::map<std::string, std::string> *result_map);

  // produce key for shared data which is always map
  // key = device_id + stamp * 100
  static bool ProduceSharedDataKey(double stamp, const std::string &device_id,
                                   std::string *key);

  // produce key for shared data which is always map
  // key = (long)(stamp * 100)*100 + device_id
  static bool ProduceSharedDataKey(double stamp, const std::string &device_id,
                                   int64_t *key);

  // conf format: param1_name=param1_value&param2_name=param2_value
  static bool ExtractParams(const std::string &conf_str,
                            const std::vector<std::string> &param_names,
                            std::vector<std::string> *param_values);

  // conf format: param1_name=param1_value&param2_name=param2_value
  static bool ExtractParam(const std::string &conf_str,
                           const std::string &param_name,
                           std::string *param_value);

 private:
  // Not allowed to instanced.
  SubnodeHelper();
  DISALLOW_COPY_AND_ASSIGN(SubnodeHelper);
};

// @brief FrameSkiper is designed for limiting the frame ratio.
class FrameSkiper {
 public:
  FrameSkiper() : min_interval_(0.0), ts_(0.0) {}

  // @brief max_ratio means max frame/s
  bool Init(const double max_ratio);

  // @brief if the frame should be skip
  // @param true skip the msg.
  bool Skip(const double ts);

 private:
  double min_interval_;
  double ts_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_
