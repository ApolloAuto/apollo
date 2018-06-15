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
//            AERROR << "not found model: " << model_name;
//            return false;
//         }
//
//         int int_value = 0;
//         if (!model_config->get_value("my_param_name", &int_value)) {
//             AERROR << "my_param_name not found."
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
#include <mutex>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include "modules/common/macro.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/cuda_util/undistortion.h"
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
    return camera_calibration_;
  }

 private:
  CalibrationConfigManager();
  ~CalibrationConfigManager();

  bool init_internal();

  friend class Singleton<CalibrationConfigManager>;

  Mutex mutex_;  // multi-thread init safe.
  bool inited_ = false;
  std::string camera_extrinsic_path_;
  std::string camera_intrinsic_path_;
  std::string radar_extrinsic_path_;
  std::string work_root_;
  CameraCalibrationPtr camera_calibration_;
  RadarCalibrationPtr radar_calibration_;

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
    return camera_intrinsic_;
  }

  inline const Eigen::Matrix<double, 4, 4>& get_camera_extrinsics() {
    return *_camera2car_pose;
  }

  inline const Eigen::Matrix<double, 4, 4>& get_car2camera_extrinsic() {
    return *_car2camera_pose;
  }

  void SetCar2CameraExtrinsicsAdj(Eigen::Matrix<double, 4, 4> matrix,
                                  bool adjusted) {
    std::lock_guard<std::mutex> lock(adj_mtx_);
    camera2car_adj_ = matrix;
    adjusted_extrinsic_ = adjusted;

    auto c_int_inv = camera_intrinsic_.block(0, 0, 3, 3).inverse();
    auto car2camera_3_4 = (camera2car_adj_.inverse()).block(0, 0, 3, 4);
    Eigen::Matrix3d camera_2car_stripped;
    camera_2car_stripped.col(0) = car2camera_3_4.col(0);
    camera_2car_stripped.col(1) = car2camera_3_4.col(1);
    camera_2car_stripped.col(2) = car2camera_3_4.col(3);

    homography_camera2car_adj_ =
     camera_2car_stripped.inverse() * c_int_inv;
  }

  bool GetCar2CameraExtrinsicsAdj(Eigen::Matrix<double, 4, 4>* matrix) {
    std::lock_guard<std::mutex> lock(adj_mtx_);
    *matrix = camera2car_adj_;
    return adjusted_extrinsic_;
  }

  inline CameraUndistortionPtr get_camera_undistort_handler() {
    return undistort_handler_;
  }

  inline CameraDistortPtr get_camera_model() { return camera_model_; }

  Eigen::Matrix<double, 3, 3> get_camera2car_homography_mat() {
    std::lock_guard<std::mutex> lock(adj_mtx_);
    if (adjusted_extrinsic_) return homography_camera2car_adj_;
    return homography_mat_;
  }

  Eigen::Matrix<double, 3, 3> get_car2camera_homography_mat() {
    return homography_mat_inverse_;
  }

  Eigen::Matrix<double, 3, 4> get_camera_projection_mat() {
    return camera_projection_mat_;
  }

  inline void get_image_height_width(int32_t* image_height,
                                     int32_t* image_width) {
    *image_height = static_cast<int32_t>(image_height_);
    *image_width = static_cast<int32_t>(image_width_);
  }

 private:
  bool init_undistortion(const std::string& intrinsics_path);
  void init_camera_model();

  Eigen::Matrix<double, 3, 4> camera_intrinsic_;  // camera intrinsic

  std::shared_ptr<Eigen::Matrix<double, 4, 4>>
      _camera2car_pose;  // camera to ego car pose
  std::shared_ptr<Eigen::Matrix<double, 4, 4>>
      _car2camera_pose;  // car to camera pose

  // Pitch angle adjusted extrinsics to ego car space on the ground
  // always available, but retreat to static one if above is false
  std::mutex adj_mtx_;
  bool adjusted_extrinsic_ = false;
  Eigen::Matrix<double, 4, 4> camera2car_adj_;
  Eigen::Matrix<double, 3, 3> homography_camera2car_adj_;

  Eigen::Matrix<double, 3, 4> camera_projection_mat_;
  Eigen::Matrix<double, 3, 3>
      homography_mat_;  // homography mat from camera 2 car
  Eigen::Matrix<double, 3, 3>
      homography_mat_inverse_;  // homography mat from car 2 camera
  volatile std::shared_ptr<Eigen::Matrix<double, 3, 3>>
      camera_homography_;  // final homography based on online calibration

  CameraUndistortionPtr undistort_handler_;
  CameraDistortPtr camera_model_;

  size_t image_height_;
  size_t image_width_;

  Eigen::Quaterniond extrinsic_quat_;
  CameraCoeffient camera_coefficient_;
};

class RadarCalibration {
 public:
  void init();
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_CALIBRATION_CONFIG_MANAGER_H_
