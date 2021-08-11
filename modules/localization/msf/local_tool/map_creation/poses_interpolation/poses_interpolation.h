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

/**
 * @file poses_interpolation.h
 * @brief
 */

#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "modules/common/util/eigen_defs.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class PosesInterpolation {
 public:
  PosesInterpolation();
  bool Init(const std::string &input_poses_path,
            const std::string &ref_timestamps_path,
            const std::string &out_poses_path,
            const std::string &extrinsic_path);
  void DoInterpolation();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void LoadPCDTimestamp();
  void WritePCDPoses();
  void PoseInterpolationByTime(
      const ::apollo::common::EigenAffine3dVec &in_poses,
      const std::vector<double> &in_timestamps,
      const std::vector<double> &ref_timestamps,
      const std::vector<unsigned int> &ref_indexes,
      std::vector<unsigned int> *out_indexes,
      std::vector<double> *out_timestamps,
      ::apollo::common::EigenAffine3dVec *out_poses);

 private:
  std::string input_poses_path_;
  std::string ref_timestamps_path_;
  std::string out_poses_path_;
  std::string extrinsic_path_;

  Eigen::Affine3d velodyne_extrinsic_;

  ::apollo::common::EigenAffine3dVec input_poses_;
  std::vector<double> input_poses_timestamps_;

  std::vector<double> ref_timestamps_;
  std::vector<unsigned int> ref_ids_;

  std::vector<unsigned int> out_indexes_;
  std::vector<double> out_timestamps_;
  ::apollo::common::EigenAffine3dVec out_poses_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
