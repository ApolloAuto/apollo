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

#include "gtest/gtest.h"
#include "ros/include/ros/ros.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

DECLARE_string(test_routing_result_file);

class OptimizerTestBase : public ::testing::Test {
 public:
  /**
   * @brief set common data settings, such as gflags path configurations.
   * If in your local test, you want to use a different file,
   * call this function first, and add the new settings.
   * For example, if you want to let FLAGS_test_routing_result_file =
   * "new_routing.pb.txt", You can implement your own SetDataConfigs by
   * ```
   * virtual void SetDataConfigs() {
   *   OptimizerTestBase::SetDataConfigs();
   *   FLAGS_test_routing_result_file = "new_routing.pb.txt"
   * }
   * ```
   */
  virtual void SetDataConfigs();
  virtual void SetUp();

  /**
   * @brief Print out the points to a file for debug and visualization purpose.
   * User can see the file, or feed it
   * into a graphic visualizer.
   */
  static void export_sl_points(
      std::vector<std::vector<common::SLPoint>>& points,
      const std::string& filename);

 protected:
  DpPolyPathConfig config_;
  Frame* frame_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
