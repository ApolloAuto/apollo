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

//
// CalibrationConfigManager use yaml config to manage all your calibration
// configs.
//
// CODE SAMPLE:
// you can use such code to access your parameters:
//
//         #include "lib/config_manager/calibration_config_manager.h"
//
//         CalibrationConfigManager* config_manager =
//               base::Singleton<CalibrationConfigManager>::get();
//
//         string model_name = "FrameClassifier";
//         const ModelConfig* model_config = NULL;
//         if (!config_manager->get_model_config(model_name, &model_config)) {
//            XLOG(ERROR) << "not found model: " << model_name;
//            return false;
//         }
//
//         int int_value = 0;
//         if (!model_config->get_value("my_param_name", &int_value)) {
//             XLOG(ERROR) << "my_param_name not found."
//             return false;
//         }
//         using int_value....
//

#ifndef MODULES_PERCEPTION_LIB_CALIBRATION_CONFIG_MANAGER_H_
#define MODULES_PERCEPTION_LIB_CALIBRATION_CONFIG_MANAGER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include "include/undistortion.h"
#include "modules/common/macro.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/obstacle/camera/common/camera.h"

namespace apollo {
namespace perception {

class CameraCalibration;
class RadarCalibration;

typedef std::shared_ptr<CameraCalibration> CameraCalibrationPtr;
typedef std::shared_ptr<RadarCalibration> RadarCalibrationPtr;
typedef std::shared_ptr<ImageGpuPreprocessHandler> CameraUndistortionPtr;
typedef std::shared_ptr<CameraDistort<double>> CameraDistortPtr;

/**
 * @brief The Basic Camera Coefficient
 */
struct CameraCoeffient {
 public:
  bool init(const std::string& camera_type,
            const std::string& camera_extrinsic_file_name,
            const std::string& camera_intrinsic_matrix_file);

 private:
  bool init_camera_extrinsic_matrix(const std::string& matrix_file);
  bool init_camera_intrinsic_matrix_and_distort_params(
      const std::string& camera_intrinsic_file);

 public:
  std::string camera_type_str;
  Eigen::Matrix4d camera_extrinsic;
  Eigen::Matrix<double, 3, 4> camera_intrinsic;
  Eigen::Matrix<double, 5, 1> distort_params;
  size_t image_height;
  size_t image_width;
};

class CalibrationConfigManager {
 public:
  // thread-safe interface.
  bool init();

  // thread-safe interface.
  bool reset();

  inline CameraCalibrationPtr get_camera_calibration() {
    return _camera_calibration;
  }

 private:
  CalibrationConfigManager();
  ~CalibrationConfigManager();

  bool init_internal();

  friend class Singleton<CalibrationConfigManager>;

  Mutex _mutex;  // multi-thread init safe.
  bool _inited = false;
  std::string _camera_extrinsic_path;
  std::string _camera_intrinsic_path;
  std::string _radar_extrinsic_path;
  std::string _work_root;
  CameraCalibrationPtr _camera_calibration;
  RadarCalibrationPtr _radar_calibration;

  DISALLOW_COPY_AND_ASSIGN(CalibrationConfigManager);
};

class CameraCalibration {
 public:
  CameraCalibration();
  ~CameraCalibration();
  bool init(const std::string& intrinsic_path,
            const std::string& extrinsic_path);

  void calculate_homographic();

  inline const Eigen::Matrix<double, 3, 4>& get_camera_intrinsic() {
    return _camera_intrinsic;
  }

  inline const Eigen::Matrix<double, 4, 4>& get_camera_extrinsics() {
    return *_camera2car_pose;
  }

  inline const Eigen::Matrix<double, 4, 4>& get_car2camera_extrinsic() {
    return *_car2camera_pose;
  }

  inline CameraUndistortionPtr get_camera_undistort_handler() {
    return _undistort_handler;
  }

  inline CameraDistortPtr get_camera_model() {
    return _camera_model;
  }

  //
  Eigen::Matrix<double, 3, 3> get_camera2car_homography_mat() {
    return _homography_mat;
  }

  Eigen::Matrix<double, 3, 3> get_car2camera_homography_mat() {
    return _homography_mat_inverse;
  }

  Eigen::Matrix<double, 3, 4> get_camera_projection_mat() {
    return _camera_projection_mat;
  }

  inline void get_image_height_width(int32_t* image_height,
                                     int32_t* image_width) {
    *image_height = static_cast<int32_t>(_image_height);
    *image_width = static_cast<int32_t>(_image_width);
  }

 private:
  bool init_undistortion(const std::string& intrinsics_path);
  void init_camera_model();

  Eigen::Matrix<double, 3, 4> _camera_intrinsic;  // camera intrinsic
  std::shared_ptr<Eigen::Matrix<double, 4, 4>>
      _camera2car_pose;  // camera to ego car pose
  std::shared_ptr<Eigen::Matrix<double, 4, 4>>
      _car2camera_pose;  // car to camera pose
  Eigen::Matrix<double, 3, 4> _camera_projection_mat;
  Eigen::Matrix<double, 3, 3>
      _homography_mat;  // homography mat from camera 2 car
  Eigen::Matrix<double, 3, 3>
      _homography_mat_inverse;  // homography mat from car 2 camera
  volatile std::shared_ptr<Eigen::Matrix<double, 3, 3>>
      _camera_homography;  // final homography based on online calibration
  CameraUndistortionPtr _undistort_handler;
  CameraDistortPtr _camera_model;
  size_t _image_height;
  size_t _image_width;
  Eigen::Quaterniond _extrinsic_quat;
  CameraCoeffient _camera_coefficient;
};

class RadarCalibration {
 public:
  void init();
};

}  // namespace perception
}  // namespace apollo

#endif  // APOLLO_PERCEPTION_LIB_CONFIG_MANAGER_H
