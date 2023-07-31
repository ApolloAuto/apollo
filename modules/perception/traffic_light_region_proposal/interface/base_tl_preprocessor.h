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
#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/pose.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace trafficlight {

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
  /**
   * @brief Construct a new base trafficlight preprocessor object.
   * 
   */
  BaseTLPreprocessor() = default;
  /**
   * @brief Destroy the base trafficlight preprocessor object.
   * 
   */
  virtual ~BaseTLPreprocessor() = default;
  /**
   * @brief Traffic light preprocessing initialization parameters.
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  virtual bool Init(const TrafficLightPreprocessorInitOptions& options) = 0;
  /**
   * @brief returns the name of the preprocessing plugin.
   * 
   * @return std::string 
   */
  virtual std::string Name() const = 0;
  /**
   * @brief Select the appropriate camera for preprocessing according to the 
            position of the vehicle and the position of the traffic light.
   * 
   * @param pose 
   * @param option 
   * @param lights 
   * @return true 
   * @return false 
   */
  virtual bool UpdateCameraSelection(
      const camera::CarPose& pose, const TLPreprocessorOption& option,
      std::vector<base::TrafficLightPtr>* lights) = 0;
  /**
   * @brief Determine whether the selected camera is available
            and synchronize timestamp information.
   * 
   * @param ts 
   * @param camera_name 
   * @return true 
   * @return false 
   */
  virtual bool SyncInformation(const double ts,
                               const std::string& camera_name) = 0;
  /**
   * @brief Update the traffic light projection status and determine
            whether the projection exceeds the boundary range.
   * 
   * @param pose 
   * @param option 
   * @param camera_name 
   * @param lights 
   * @return true 
   * @return false 
   */
  virtual bool UpdateLightsProjection(
      const camera::CarPose& pose, const TLPreprocessorOption& option,
      const std::string& camera_name,
      std::vector<base::TrafficLightPtr>* lights) = 0;
  /**
   * @brief Set the camera working flag object.
   * 
   * @param camera_name 
   * @param is_working 
   * @return true 
   * @return false 
   */
  virtual bool SetCameraWorkingFlag(const std::string& camera_name,
                                    bool is_working) = 0;
  /**
   * @brief Get the camera working flag object.
   * 
   * @param camera_name 
   * @param is_working 
   * @return true 
   * @return false 
   */
  virtual bool GetCameraWorkingFlag(const std::string& camera_name,
                                    bool* is_working) const = 0;
  /**
   * @brief Get the camera names by descending focal len object.
   * 
   * @return const std::vector<std::string>& 
   */
  virtual const std::vector<std::string>& GetCameraNamesByDescendingFocalLen()
      const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTLPreprocessor);
};

PERCEPTION_REGISTER_REGISTERER(BaseTLPreprocessor);
#define PERCEPTION_REGISTER_TLPREPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseTLPreprocessor, name)

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
