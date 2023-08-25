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

#pragma once

#include <memory>
#include <vector>

#include "modules/perception/multi_sensor_fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {

class Scene {
 public:
  Scene();
  ~Scene();

  /**
   * @brief Get the foreground track objects
   *
   * @return std::vector<TrackPtr>&
   */
  inline std::vector<TrackPtr>& GetForegroundTracks() {
    return foreground_tracks_;
  }

  /**
   * @brief Get the foreground track objects
   *
   * @return const std::vector<TrackPtr>&
   */
  inline const std::vector<TrackPtr>& GetForegroundTracks() const {
    return foreground_tracks_;
  }

  /**
   * @brief Get the background track objects
   *
   * @return std::vector<TrackPtr>&
   */
  inline std::vector<TrackPtr>& GetBackgroundTracks() {
    return background_tracks_;
  }

  /**
   * @brief Get the background track objects
   *
   * @return const std::vector<TrackPtr>&
   */
  inline const std::vector<TrackPtr>& GetBackgroundTracks() const {
    return background_tracks_;
  }

  /**
   * @brief Add a foreground track object
   *
   * @param track foreground track object
   */
  void AddForegroundTrack(TrackPtr track);

  /**
   * @brief Add a background track object
   *
   * @param track background track object
   */
  void AddBackgroundTrack(TrackPtr track);

 protected:
  std::vector<TrackPtr> foreground_tracks_;
  std::vector<TrackPtr> background_tracks_;
};

typedef std::shared_ptr<Scene> ScenePtr;
typedef std::shared_ptr<const Scene> SceneConstPtr;

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
