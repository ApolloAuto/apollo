/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/camera_detection_occupancy/tracker/common/camera_track.h"
#include "modules/perception/common/base/frame.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraTrackManager {
 public:
  CameraTrackManager() = default;
  ~CameraTrackManager() = default;

  /**
   * @brief Get tracked targets, which can modify data.
   *
   * @return std::vector<CameraTrackPtr>&
   */
  inline std::vector<CameraTrackPtr> &mutable_tracks() { return tracks_; }

  /**
   * @brief Get the Tracks object
   *
   * @return const std::vector<CameraTrackPtr>&
   */
  inline const std::vector<CameraTrackPtr> &GetTracks() const {
    return tracks_;
  }

  /**
   * @brief Add Camera tracked object.
   *
   * @param track
   */
  void AddTrack(const CameraTrackPtr &track) { tracks_.push_back(track); }

  /**
   * @brief Remove already lost targets.
   *
   * @return int
   */
  int RemoveLostTracks();

  /**
   * @brief Clear Camera tracked objects.
   *
   */
  void ClearTracks();

 protected:
  std::vector<CameraTrackPtr> tracks_;

 private:
  DISALLOW_COPY_AND_ASSIGN(CameraTrackManager);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
