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
#include "modules/perception/lane_detection/app/debug_info.h"

#include <iomanip>

#include "cyber/common/log.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"

namespace apollo {
namespace perception {
namespace camera {

int WriteLanelines(const bool enabled, const std::string &save_path,
                   const std::vector<base::LaneLine> &lane_objects) {
  if (!enabled) {
    return -1;
  }
  FILE *file_save = fopen(save_path.data(), "wt");
  if (file_save == nullptr) {
    AERROR << "Failed to open lane save path: " << save_path;
    return -1;
  }
  int lane_line_size = static_cast<int>(lane_objects.size());
  AINFO << "Lane line num: " << lane_line_size;
  fprintf(file_save, "[\n");
  for (int j = 0; j < lane_line_size; ++j) {
    const base::LaneLineCubicCurve &curve_camera =
        lane_objects[j].curve_camera_coord;
    const base::LaneLineCubicCurve &curve_img =
        lane_objects[j].curve_image_coord;
    const std::vector<base::Point3DF> &camera_point_set =
        lane_objects[j].curve_camera_point_set;
    const std::vector<base::Point2DF> &image_point_set =
        lane_objects[j].curve_image_point_set;
    fprintf(file_save, "{\n");
    fprintf(file_save, "\"type\":%d,\n", lane_objects[j].type);
    fprintf(file_save, "\"pos_type\":%d,\n", lane_objects[j].pos_type);
    // Camera curve
    fprintf(file_save, "\"camera_curve\":\n");
    fprintf(file_save, "{\"a\":%.10f,\"b\":%.10f,\"c\":%.10f,\"d\":%.10f,",
            curve_camera.a, curve_camera.b, curve_camera.c, curve_camera.d);
    fprintf(file_save, "\"x0\":%.10f,\"x1\":%.10f},\n", curve_camera.x_start,
            curve_camera.x_end);
    // Camera image point set
    fprintf(file_save, "\"camera_point_set\":\n");
    fprintf(file_save, "[");
    for (size_t k = 0; k < camera_point_set.size(); k++) {
      if (k < camera_point_set.size() - 1) {
        fprintf(file_save, "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},",
                camera_point_set[k].x, camera_point_set[k].y,
                camera_point_set[k].z);
      } else {
        fprintf(file_save, "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
                camera_point_set[k].x, camera_point_set[k].y,
                camera_point_set[k].z);
      }
    }
    fprintf(file_save, "],");
    fprintf(file_save, "\n");
    // Image curve
    fprintf(file_save, "\"image_curve\":\n");
    fprintf(file_save, "{\"a\":%.10f,\"b\":%.10f,\"c\":%.10f,\"d\":%.10f,",
            curve_img.a, curve_img.b, curve_img.c, curve_img.d);
    fprintf(file_save, "\"x0\":%.10f,\"x1\":%.10f},\n", curve_img.x_start,
            curve_img.x_end);
    // Curve image point set
    fprintf(file_save, "\"image_point_set\":\n");
    fprintf(file_save, "[");
    for (size_t k = 0; k < image_point_set.size(); ++k) {
      if (k < image_point_set.size() - 1) {
        fprintf(file_save, "{\"x\":%.4f,\"y\":%.4f},", image_point_set[k].x,
                image_point_set[k].y);
      } else {
        fprintf(file_save, "{\"x\":%.4f,\"y\":%.4f}", image_point_set[k].x,
                image_point_set[k].y);
      }
    }
    fprintf(file_save, "]");
    fprintf(file_save, "\n");
    if (j < lane_line_size - 1) {
      fprintf(file_save, "},\n");
    } else {
      fprintf(file_save, "}\n");
    }
  }
  fprintf(file_save, "]\n");
  fclose(file_save);
  return 0;
}

int WriteCalibrationOutput(bool enabled, const std::string &out_path,
                           const CameraFrame *frame) {
  if (!enabled) {
    return -1;
  }
  float pitch_angle = 0.f;
  float camera_ground_height = 0.f;
  if (!frame->calibration_service->QueryCameraToGroundHeightAndPitchAngle(
          &camera_ground_height, &pitch_angle)) {
    AERROR << "Failed to query camera to ground height and pitch.";
    return -1;
  }

  FILE *file_save = fopen(out_path.data(), "wt");
  if (file_save == nullptr) {
    AERROR << "Failed to open output path: " << out_path.data();
    return -1;
  }
  fprintf(file_save, "camera_ground_height:\t%f\n", camera_ground_height);
  fprintf(file_save, "pitch_angle:\t%f\n", pitch_angle);
  fclose(file_save);
  return 0;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
