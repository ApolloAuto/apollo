/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cyber/component/component.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/v2x/fusion/apps/common/trans_tools.h"
#include "modules/v2x/fusion/configs/ft_config_manager.h"
#include "modules/v2x/fusion/configs/fusion_tracker_gflags.h"
#include "modules/v2x/fusion/libs/fusion/fusion.h"

namespace apollo {
namespace v2x {
namespace ft {

class V2XFusionComponent
    : public apollo::cyber::Component<PerceptionObstacles> {
 public:
  ~V2XFusionComponent();

  std::string Name() const;

  bool Init() override;

  bool Proc(const std::shared_ptr<PerceptionObstacles>& perception_obstacles)
      override;

 private:
  bool V2XMessageFusionProcess(
      const std::shared_ptr<PerceptionObstacles>& perception_obstacles);
  void SerializeMsg(const std::vector<base::Object>& objects,
                    std::shared_ptr<PerceptionObstacles> obstacles);
  Fusion fusion_;

  std::shared_ptr<apollo::cyber::Reader<LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<apollo::cyber::Reader<V2XObstacles>> v2x_obstacles_reader_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>>
      perception_fusion_obstacles_writer_;
  apollo::common::Header header_;
};

CYBER_REGISTER_COMPONENT(V2XFusionComponent)

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
