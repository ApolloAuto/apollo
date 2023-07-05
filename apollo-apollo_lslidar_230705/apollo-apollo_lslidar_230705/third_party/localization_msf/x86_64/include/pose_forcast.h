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

#pragma once

#include "sins_struct.h"

namespace apollo {
namespace localization {
namespace msf {

class PoseForcastImpl;
class PoseForcast {
 public:
  PoseForcast();
  ~PoseForcast();

  void SetMaxListNum(int n);
  void SetMaxAccelInput(double threshold);
  void SetMaxGyroInput(double threshold);
  void SetWheelExtrinsic(double x, double y, double z,
                         double qx, double qy, double qz, double qw);
  void SetZoneId(int zone_id);
  
  double GetLastestImuTime();

  void PushInspvaData(const InsPva &data);
  void PushImuData(const ImuData &data);
  void PushWheelSpeedData(const WheelspeedData &data);

  int GetBestForcastPose(double time, double init_time,
                         const Pose &init_pose, Pose *forcast_pose);

  bool GetImuframeDeltaPose(const double& start_time, const double& end_time, TransformD& delta_pose);

 protected:
  PoseForcastImpl *pose_forcast_impl_;
};

}
}
}