/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/camera_detection_occupancy/interface/base_filter.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/base/object_pool_types.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraTrack {
 public:
  /**
   * @brief Construct a new Camera Track object
   *
   * @param obs
   * @param timestamp
   */
  CameraTrack(const base::ObjectPtr &obs, const double timestamp);
  ~CameraTrack() {}

  /**
   * @brief update the object after association with a Camera obervation
   *
   * @param obs_camera
   * @param timestamp
   */
  void UpdataObsCamera(const base::ObjectPtr &obs_camera,
                       const double timestamp);

  /**
   * @brief Set the Obs Camera Nullptr object
   *
   */
  void SetObsCameraNullptr();

  /**
   * @brief Get the Obs Id object
   *
   * @return int
   */
  int GetObsId() const;

  /**
   * @brief Get the Obs Camera object
   *
   * @return base::ObjectPtr
   */
  base::ObjectPtr GetObsCamera();

  /**
   * @brief Get the Obs object
   *
   * @return base::ObjectPtr
   */
  base::ObjectPtr GetObs();

  /**
   * @brief Get the Timestamp object
   *
   * @return double
   */
  double GetTimestamp();

  /**
   * @brief Get the Tracking Time object
   *
   * @return double
   */
  double GetTrackingTime();

  /**
   * @brief Track state is dead or not
   *
   * @return true
   * @return false
   */
  bool IsDead() { return is_dead_; }

  /**
   * @brief Set the Dead object
   *
   */
  void SetDead() { is_dead_ = true; }

  bool IsAssigned() { return is_assigned_; }

  void SetUnAssigned() { is_assigned_ = false; }

  /**
   * @brief If the target tracking time is greater than the set threshold,
   * which means that the target has been tracked.
   *
   * @return true
   * @return false
   */
  bool ConfirmTrack() { return tracked_times_ > s_tracked_times_threshold_; }

  /**
   * @brief Set the Tracked Times Threshold object
   *
   * @param threshold
   */
  static void SetTrackedTimesThreshold(const int &threshold) {
    s_tracked_times_threshold_ = threshold;
  }

  /**
   * @brief Set the Chosen Filter object
   *
   * @param chosen_filter
   */
  static void SetChosenFilter(const std::string &chosen_filter) {
    s_chosen_filter_ = chosen_filter;
  }

  /**
   * @brief Set the Use Filter object
   *
   * @param use_filter
   */
  static void SetUseFilter(bool use_filter) { s_use_filter_ = use_filter; }

 private:
  double timestamp_ = 0.0;
  int obs_id_ = 0;
  int tracked_times_ = 0;
  double tracking_time_ = 0.0;
  bool is_dead_ = false;
  bool is_assigned_ = false;
  base::ObjectPtr obs_camera_ = nullptr;  // observasion from Camera
  base::ObjectPtr obs_ = nullptr;         // track result after tracking
  std::shared_ptr<BaseFilter> filter_ = nullptr;

  static std::string s_chosen_filter_;
  static int s_current_idx_;
  static int s_tracked_times_threshold_;
  static bool s_use_filter_;

  DISALLOW_COPY_AND_ASSIGN(CameraTrack);
};

typedef std::shared_ptr<CameraTrack> CameraTrackPtr;

}  // namespace camera
}  // namespace perception
}  // namespace apollo
