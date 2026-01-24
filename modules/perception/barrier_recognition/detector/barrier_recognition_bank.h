/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <string>
#include <map>

#include "cyber/common/log.h"
#include "cyber/common/file.h"

#include "modules/perception/common/util.h"
#include "modules/perception/barrier_recognition/interface/base_barrier_recognizer.h"

namespace apollo {
namespace perception {
namespace lidar {

class BarrierRecognizerBank {
public:
    BarrierRecognizerBank() = default;
    virtual ~BarrierRecognizerBank() = default;

    /**
     * @brief Init of BarrierRecognizerBank object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const BarrierRecognizerInitOptions& options = BarrierRecognizerInitOptions());

    /**
     * @brief calculate the relative position of the vehicle based on the reflector
     *
     * @param options object filter options
     * @param message pointcloud message
     * @param frame lidar frame to filter
     * @param dock_relative_points vehicle position relative to reflectors
     * @return true
     * @return false
     */
    bool Recognize(const BarrierRecognizerOptions& options, 
                   LidarFrame *frame, float& open_percent);

    /**
     * @brief Name of the class
     *
     * @return std::string name
     */
    std::string Name() const { return "BarrierRecognizer"; }

    /**
     * @brief Number of recognizer banks
     * 
     * @return std::size_t 
     */
  std::size_t Size() const { return barrier_bank_map_.size(); }

private:
  std::map<std::string, std::shared_ptr<BaseBarrierRecognizer>>
      barrier_bank_map_;

  DISALLOW_COPY_AND_ASSIGN(BarrierRecognizerBank);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
