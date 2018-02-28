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

#include "modules/perception/obstacle/radar/modest/conti_radar_util.h"

namespace apollo {
namespace perception {

bool ContiRadarUtil::IsFp(const ContiRadarObs &contiobs,
                          const ContiParams &params, const int delay_frames,
                          const int tracking_times) {
  int cls = contiobs.obstacle_class();
  if (tracking_times < delay_frames * 2) {
    const double &lo_vel_rms = contiobs.longitude_vel_rms();
    const double &la_vel_rms = contiobs.lateral_vel_rms();
    const double &lo_dist_rms = contiobs.longitude_dist_rms();
    const double &la_dist_rms = contiobs.lateral_dist_rms();
    const double &probexist = contiobs.probexist();
    if (cls == static_cast<int>(ContiObjectType::CONTI_CAR) ||
        cls == static_cast<int>(ContiObjectType::CONTI_TRUCK)) {
      if (probexist < params.probexist_vehicle) {
        return true;
      } else if (lo_vel_rms > params.lo_vel_rms_vehicle ||
                 la_vel_rms > params.la_vel_rms_vehicle ||
                 lo_dist_rms > params.lo_dist_rms_vehicle ||
                 la_dist_rms > params.la_dist_rms_vehicle) {
        return true;
      } else if (tracking_times <= delay_frames) {
        return true;
      }
    } else if (cls == static_cast<int>(ContiObjectType::CONTI_PEDESTRIAN)) {
      if (probexist < params.probexist_pedestrian) {
        return true;
      } else if (lo_vel_rms > params.lo_vel_rms_pedestrian ||
                 la_vel_rms > params.la_vel_rms_pedestrian ||
                 lo_dist_rms > params.lo_dist_rms_pedestrian ||
                 la_dist_rms > params.la_dist_rms_pedestrian) {
        return true;
      }
    } else if (cls == static_cast<int>(ContiObjectType::CONTI_MOTOCYCLE) ||
               cls == static_cast<int>(ContiObjectType::CONTI_BICYCLE)) {
      if (probexist < params.probexist_bicycle) {
        return true;
      } else if (lo_vel_rms > params.lo_vel_rms_bicycle ||
                 la_vel_rms > params.la_vel_rms_bicycle ||
                 lo_dist_rms > params.lo_dist_rms_bicycle ||
                 la_dist_rms > params.la_dist_rms_bicycle) {
        return true;
      }
    } else if (cls == static_cast<int>(ContiObjectType::CONTI_POINT) ||
               cls == static_cast<int>(ContiObjectType::CONTI_WIDE) ||
               cls == static_cast<int>(ContiObjectType::CONTI_UNKNOWN)) {
      if (probexist < params.probexist_unknown) {
        return true;
      } else if (lo_vel_rms > params.lo_vel_rms_unknown ||
                 la_vel_rms > params.la_vel_rms_unknown ||
                 lo_dist_rms > params.lo_dist_rms_unknown ||
                 la_dist_rms > params.la_dist_rms_unknown) {
        return true;
      } else if (tracking_times <= delay_frames) {
        return true;
      }
    }
  }
  int meas_state = contiobs.meas_state();
  if (meas_state == static_cast<int>(ContiMeasState::CONTI_DELETED) ||
      meas_state == static_cast<int>(ContiMeasState::CONTI_PREDICTED) ||
      meas_state == static_cast<int>(ContiMeasState::CONTI_DELETED_FOR)) {
    return true;
  }
  return false;
}

}  // namespace perception
}  // namespace apollo
