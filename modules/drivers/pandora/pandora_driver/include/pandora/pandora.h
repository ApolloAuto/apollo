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

#ifndef INCLUDE_PANDORA_PANDORA_H_
#define INCLUDE_PANDORA_PANDORA_H_

#include <boost/function.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>
#include <vector>

#include "pandar40p/pandar40p.h"
#include "pandar40p/point_types.h"

namespace apollo {
namespace drivers {
namespace hesai {

class Pandora_Internal;

class CameraCalibration {
 public:
  //  intrinsic
  cv::Mat cameraK;
  cv::Mat cameraD;
  //  extrinsic
  std::vector<double> cameraT;  //  w x y z
  std::vector<double> cameraR;  //  x y z
};

class Pandora {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   *        pandoraCameraPort The port of camera data
   *        cameraCallback    the call back for camera data
   */
  Pandora(std::string device_ip, const uint16_t lidar_port,
          const uint16_t gps_port,
          boost::function<void(boost::shared_ptr<PPointCloud>, double)>
              pcl_callback,
          boost::function<void(double)> gps_callback, uint16_t start_angle,
          const uint16_t pandoraCameraPort,
          boost::function<void(boost::shared_ptr<cv::Mat> matp,
                               double timestamp, int picid, bool distortion)>
              cameraCallback,
          bool enable_camera = true, int tz = 0,
          std::string frame_id = std::string("hesai40"));
  /**
   * @brief deconstructor
   */
  ~Pandora();

  /**
   * @brief load the lidar correction file
   * @param contents The correction contents of lidar correction
   */
  int LoadLidarCorrectionFile(std::string contents);

  /**
   * @brief Reset Lidar's start angle.
   * @param angle The start angle
   */
  void ResetLidarStartAngle(uint16_t start_angle);

  /**
   * @brief Upload the camera calibration contents to Pandora Device.
   * @param calibs calibration contents , include camera intrinsics and
   * extrinsics.
   */
  int UploadCameraCalibration(const CameraCalibration calibs[5]);

  /**
   * @brief Get Camera's Calibration, include camera intrinsics and extrinsics.
   * @param calibs calibration contents , include camera intrinsics and
   * extrinsics.
   */
  int GetCameraCalibration(CameraCalibration calibs[5]);

  /**
   * @brief Reset camera calibration as factory-set.
   */
  int ResetCameraClibration();

  /**
   * @brief Run SDK.
   */
  int Start();

  /**
   * @brief Stop SDK.
   */
  void Stop();

 private:
  Pandora_Internal *internal_;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // INCLUDE_PANDORA_PANDORA_H_
