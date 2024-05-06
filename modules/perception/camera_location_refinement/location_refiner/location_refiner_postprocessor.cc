/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_location_refinement/location_refiner/location_refiner_postprocessor.h"  // NOLINT

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/i_lib/core/i_constant.h"
#include "modules/perception/common/camera/common/global_config.h"
#include "modules/perception/common/util.h"
// TODO(Xun): code completion

namespace apollo {
namespace perception {
namespace camera {

LocationRefinerPostprocessor::LocationRefinerPostprocessor() {
  postprocessor_.reset(new ObjPostProcessor);
}

bool LocationRefinerPostprocessor::Init(
    const PostprocessorInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  if (!cyber::common::GetProtoFromFile(config_file, &location_refiner_param_)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  calibration_service_ = options.calibration_service;
  ACHECK(calibration_service_ != nullptr);
  return true;
}

bool LocationRefinerPostprocessor::Process(const PostprocessorOptions &options,
                                           onboard::CameraFrame *frame) {
  if (frame->detected_objects.empty() ||
      !options.do_refinement_with_calibration_service) {
    ADEBUG << "Do not run obstacle postprocessor.";
    return true;
  }

  Eigen::Vector4d plane;
  if (!calibration_service_->QueryGroundPlaneInCameraFrame(&plane)) {
    AINFO << "No valid ground plane in the service.";
  }

  float query_plane[4] = {
      static_cast<float>(plane(0)), static_cast<float>(plane(1)),
      static_cast<float>(plane(2)), static_cast<float>(plane(3))};

  const auto &camera_k_matrix = frame->camera_k_matrix;
  float k_mat[9] = {0};
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      k_mat[i * 3 + j] = camera_k_matrix(i, j);
    }
  }
  AINFO << "Camera k matrix input to obstacle postprocessor: \n"
        << k_mat[0] << ", " << k_mat[1] << ", " << k_mat[2] << "\n"
        << k_mat[3] << ", " << k_mat[4] << ", " << k_mat[5] << "\n"
        << k_mat[6] << ", " << k_mat[7] << ", " << k_mat[8] << "\n";

  const int width_image = frame->data_provider->src_width();
  const int height_image = frame->data_provider->src_height();
  postprocessor_->Init(k_mat, width_image, height_image);

  ObjPostProcessorOptions obj_postprocessor_options;
  int nr_valid_obj = 0;

  for (auto &obj : frame->detected_objects) {
    ++nr_valid_obj;
    float object_center[3] = {obj->camera_supplement.local_center(0),
                              obj->camera_supplement.local_center(1),
                              obj->camera_supplement.local_center(2)};
    float bbox2d[4] = {
        obj->camera_supplement.box.xmin, obj->camera_supplement.box.ymin,
        obj->camera_supplement.box.xmax, obj->camera_supplement.box.ymax};

    float bottom_center[2] = {(bbox2d[0] + bbox2d[2]) / 2, bbox2d[3]};
    float h_down = (static_cast<float>(height_image) - k_mat[5]) *
                   location_refiner_param_.roi_h2bottom_scale();
    bool is_in_rule_roi =
        is_in_roi(bottom_center, static_cast<float>(width_image),
                  static_cast<float>(height_image), k_mat[5], h_down);
    float dist2camera = algorithm::ISqrt(algorithm::ISqr(object_center[0]) +
                                         algorithm::ISqr(object_center[2]));

    if (dist2camera > location_refiner_param_.min_dist_to_camera() ||
        !is_in_rule_roi) {
      ADEBUG << "Pass for obstacle postprocessor.";
      continue;
    }

    float dimension_hwl[3] = {obj->size(2), obj->size(1), obj->size(0)};
    float box_cent_x = (bbox2d[0] + bbox2d[2]) / 2;
    Eigen::Vector3f image_point_low_center(box_cent_x, bbox2d[3], 1);
    Eigen::Vector3f point_in_camera =
        static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
            camera_k_matrix.inverse() * image_point_low_center);
    float theta_ray =
        static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
    float rotation_y =
        theta_ray + static_cast<float>(obj->camera_supplement.alpha);

    // enforce the ry to be in the range [-pi, pi)
    const float PI = algorithm::Constant<float>::PI();
    if (rotation_y < -PI) {
      rotation_y += 2 * PI;
    } else if (rotation_y >= PI) {
      rotation_y -= 2 * PI;
    }

    // process
    memcpy(obj_postprocessor_options.bbox, bbox2d, sizeof(float) * 4);
    obj_postprocessor_options.check_lowerbound = true;
    camera::LineSegment2D<float> line_seg(bbox2d[0], bbox2d[3], bbox2d[2],
                                          bbox2d[3]);
    obj_postprocessor_options.line_segs.push_back(line_seg);
    memcpy(obj_postprocessor_options.hwl, dimension_hwl, sizeof(float) * 3);
    obj_postprocessor_options.ry = rotation_y;
    // refine with calibration service, support ground plane model currently
    // {0.0f, cos(tilt), -sin(tilt), -camera_ground_height}
    memcpy(obj_postprocessor_options.plane, query_plane, sizeof(float) * 4);

    // changed to touching-ground center
    object_center[1] += dimension_hwl[0] / 2;
    postprocessor_->PostProcessObjWithGround(
        obj_postprocessor_options, object_center, dimension_hwl, &rotation_y);
    object_center[1] -= dimension_hwl[0] / 2;

    float z_diff_camera =
        object_center[2] - obj->camera_supplement.local_center(2);

    // fill back results
    obj->camera_supplement.local_center(0) = object_center[0];
    obj->camera_supplement.local_center(1) = object_center[1];
    obj->camera_supplement.local_center(2) = object_center[2];

    obj->center(0) = static_cast<double>(object_center[0]);
    obj->center(1) = static_cast<double>(object_center[1]);
    obj->center(2) = static_cast<double>(object_center[2]);
    obj->center = frame->camera2world_pose * obj->center;

    AINFO << "diff on camera z: " << z_diff_camera;
    AINFO << "Obj center from postprocessor: " << obj->center.transpose();
  }

  return true;
}

// Register plugin.
REGISTER_OBSTACLE_POSTPROCESSOR(LocationRefinerPostprocessor);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
