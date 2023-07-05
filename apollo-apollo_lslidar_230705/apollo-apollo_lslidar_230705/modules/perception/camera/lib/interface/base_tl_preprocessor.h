/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/camera/lib/traffic_light/preprocessor/pose.h"
#include "modules/perception/base/traffic_light.h"

namespace apollo {
namespace perception {
namespace camera {

struct TrafficLightPreprocessorInitOptions : public BaseInitOptions {
  int gpu_id = 0;
  float sync_interval_seconds;
  std::vector<std::string> camera_names;
};

struct TLPreprocessorOption {
  std::map<std::string, int>* image_borders_size = nullptr;
};

class BaseTLPreprocessor {
 public:
  BaseTLPreprocessor() = default;
  virtual ~BaseTLPreprocessor() = default;

  virtual bool Init(const TrafficLightPreprocessorInitOptions& options) = 0;

  virtual std::string Name() const = 0;

  virtual bool UpdateCameraSelection(const CarPose& pose,
                             const TLPreprocessorOption& option,
                             std::vector<base::TrafficLightPtr>* lights) = 0;

  virtual bool SyncInformation(const double ts,
                                const std::string& camera_name) = 0;

  virtual bool UpdateLightsProjection(const CarPose& pose,
                              const TLPreprocessorOption& option,
                              const std::string& camera_name,
                              std::vector<base::TrafficLightPtr>* lights) = 0;

  virtual bool SetCameraWorkingFlag(const std::string& camera_name,
                                    bool is_working) = 0;

  virtual bool GetCameraWorkingFlag(const std::string& camera_name,
                            bool* is_working) const = 0;

  virtual const std::vector<std::string>&
                GetCameraNamesByDescendingFocalLen() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTLPreprocessor);
};

PERCEPTION_REGISTER_REGISTERER(BaseTLPreprocessor);
#define PERCEPTION_REGISTER_TLPREPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseTLPreprocessor, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
