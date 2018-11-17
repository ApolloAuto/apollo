/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_
#define MODULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
namespace perception {

class SubnodeHelper {
 public:
  // subnode has a field named reserve
  // when reserve like "source_name:./data;source_param:.fsi"
  // you can use this func to get the unordered_map
  static bool ParseReserveField(
      const std::string &reserve,
      std::unordered_map<std::string, std::string> *result_map);

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

#endif  // MODULES_PERCEPTION_ONBOARD_SUBNODE_HELPER_H_
