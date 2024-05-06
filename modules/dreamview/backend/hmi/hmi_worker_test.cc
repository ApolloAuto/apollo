/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/hmi/hmi_worker.h"
#include "modules/dreamview/backend/common/util/hmi_util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace dreamview {

TEST(HMIWorker, LoadConfigAndMode) {
  const HMIConfig config =
      util::HMIUtil::LoadConfig(FLAGS_dv_hmi_modes_config_path);
  for (const auto& iter : config.modes()) {
    const std::string& mode_conf_file = iter.second;
    const HMIMode& mode = util::HMIUtil::LoadMode(mode_conf_file);
    EXPECT_FALSE(mode.modules().empty())
        << "No HMI module loaded from " << mode_conf_file;
  }
}

}  // namespace dreamview
}  // namespace apollo
