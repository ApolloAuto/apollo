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

namespace apollo {
namespace localization {
namespace msf {

class LidarLocatorImpl;
class LidarLocator {
 public:
  LidarLocator();
  ~LidarLocator();
 public:
  void Init(unsigned int filter_size_x, unsigned int filter_size_y,
            float resolution, unsigned int node_size_x,
            unsigned int node_size_y);

  void SetVelodyneExtrinsic(double x, double y, double z,
                              double qx, double qy, double qz, double qw);
//   void SetVehicleHeight(double height);
  void SetValidThreshold(float valid_threashold);
  void SetImageAlignMode(int mode);
  void SetLocalizationMode(int mode);

  void SetDeltaYawLimit(double limit);
  void SetDeltaPitchRollLimit(double limit);

 public:
  // set map node data
  void SetMapNodeData(int width, int height, int level_num,
      const float *const*intensities, const float *const*intensities_var,
      const float *const*altitudes, const unsigned int *const*counts);
  void SetMapNodeLeftTopCorner(double x, double y);

  // set point cloud
  void SetPointCloudData(int size, const double* pt_xs, const double* pt_ys, 
      const double* pt_zs, const unsigned char* intensities);

  // compute localization result
  int Compute(double pose_x, double pose_y, double pose_z,
              double pose_qx, double pose_qy, double pose_qz,
              double pose_qw, bool use_avx = false);

 public:
  /**@brief Get the current optimal pose result. */
  void GetLocationPose(double *x, double *y, double *z,
                double *qx, double *qy, double *qz, double *qw);
  
  /**@brief Get the covariance of the current optimal location. */
  void GetLocationCovariance(const double **data, int *width, int *height);

  void GetSSDDistribution(const double **data, int *width, int *height);

  double GetLocationScore() const;
  
 protected:
  LidarLocatorImpl* _lidar_locator;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo