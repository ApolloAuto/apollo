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
#include "modules/perception/camera/app/debug_info.h"

#include <iomanip>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

std::vector<std::string> type_string = {
    "Unknown", "Unknown", "Unknown", "Pedestrian", "Cyclist", "Car", "Unknown"};

std::vector<std::string> sub_type_string = {"UNKNOWN",
                                            "UNKNOWN_MOVABLE",
                                            "UNKNOWN_UNMOVABLE",
                                            "CAR",
                                            "VAN",
                                            "TRUCK",
                                            "BUS",
                                            "CYCLIST",
                                            "MOTORCYCLIST",
                                            "TRICYCLIST",
                                            "PEDESTRIAN",
                                            "TRAFFICCONE",
                                            "MAX_OBJECT_TYPE"};

void WriteCamera2World(std::ofstream &fout, int frame_num,
                       const Eigen::Affine3d &pose) {
  if (!fout.is_open()) {
    AERROR << "Cannot write Camera2World!";
    return;
  }
  fout << frame_num;
  // Save the current format flags
  const auto old_flags = fout.flags();
  // Save previous precision setting
  const auto old_precision = fout.precision();
  fout << std::fixed << std::setprecision(9);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      fout << " " << pose(i, j);
    }
  }
  fout << std::endl;
  // Restore the output format flags
  fout.flags(old_flags);
  // Restore the precision
  fout.precision(old_precision);
}

void WriteTracking(std::ofstream &fout, int frame_num,
                   const std::vector<base::ObjectPtr> &tracked_object) {
  if (!fout.is_open()) {
    AERROR << "Cannot write tracking!";
    return;
  }
  char output[500];
  for (size_t i = 0; i < tracked_object.size(); ++i) {
    base::ObjectPtr ptr = tracked_object[i];
    snprintf(output, sizeof(output),
             "%d %d %s -1 -1 %2.3f %4.3f %4.3f %4.3f %4.3f "
             "%2.6f %2.6f %2.6f %2.3f %2.3f %2.3f %2.3f %.3f "
             "%2.6f %2.6f %2.6f %.30s",
             frame_num, ptr->track_id,
             sub_type_string[static_cast<int>(ptr->sub_type)].c_str(),
             ptr->camera_supplement.alpha, ptr->camera_supplement.box.xmin,
             ptr->camera_supplement.box.ymin, ptr->camera_supplement.box.xmax,
             ptr->camera_supplement.box.ymax, ptr->size[2], ptr->size[1],
             ptr->size[0], ptr->center[0], ptr->center[1], ptr->center[2],
             ptr->theta, ptr->type_probs[static_cast<int>(ptr->type)],
             ptr->velocity[0], ptr->velocity[1], ptr->velocity[2],
             ptr->camera_supplement.sensor_name.c_str());
    fout << output << std::endl;
  }
}

int WriteDetections(const bool enabled, const std::string &out_path,
                    const std::vector<base::ObjectPtr> &objects) {
  if (!enabled) {
    return -1;
  }
  std::ofstream outf(out_path, std::ofstream::out);
  AINFO << "Write detection to: " << out_path;
  if (!outf.is_open()) {
    AERROR << "Cannot open output file: " << out_path;
    return -1;
  }

  // 1    type         Describes the type of object: 'Car', 'Van', 'Truck',
  //                   'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
  //                   'Misc' or 'DontCare'
  // 1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
  //                   truncated refers to the object leaving image boundaries
  // 1    occluded     Integer (0,1,2,3) indicating occlusion state:
  //                   0 = fully visible, 1 = partly occluded
  //                   2 = largely occluded, 3 = unknown
  // 1    alpha        Observation angle of object, ranging [-pi..pi]
  // 4    bbox         2D bounding box of object in the image (0-based index):
  //                   contains left, top, right, bottom pixel coordinates
  // 3    dimensions   3D object dimensions: height, width, length (in meters)
  // 3    location     3D object location x,y,z in camera coordinates (in
  // meters) 1    rotation_y   Rotation ry around Y-axis in camera coordinates
  // [-pi..pi] 1  score        Only for results: Float, indicating confidence
  // in detection, needed for p/r curves, higher is better.
  for (const auto &obj : objects) {
    switch (obj->type) {
      case base::ObjectType::VEHICLE:
        outf << "Car";
        break;
      case base::ObjectType::BICYCLE:
        outf << "Cyclist";
        break;
      case base::ObjectType::PEDESTRIAN:
        outf << "Pedestrian";
        break;
      default:
        AERROR << "Unknown object type: " << static_cast<int>(obj->type);
        outf << "Unknown";
    }

    outf << " 0 0 ";

    outf << obj->camera_supplement.alpha;
    outf << " " << obj->camera_supplement.box.xmin;
    outf << " " << obj->camera_supplement.box.ymin;
    outf << " " << obj->camera_supplement.box.xmax;
    outf << " " << obj->camera_supplement.box.ymax;
    outf << " " << obj->size[2];
    outf << " " << obj->size[1];
    outf << " " << obj->size[0];
    outf << " " << obj->center[0];
    outf << " " << obj->center[1];
    outf << " " << obj->center[2];
    outf << " " << obj->theta;
    outf << " " << obj->type_probs[static_cast<int>(obj->type)];
    //    if (FLAGS_with_front) {
    //      outf << " " << obj->front_upper_left[0];
    //      outf << " " << obj->front_upper_left[1];
    //      outf << " " << obj->front_lower_right[0];
    //      outf << " " << obj->front_lower_right[1];
    //    }
    //    if (FLAGS_with_rear) {
    //      outf << " " << obj->back_upper_left[0];
    //      outf << " " << obj->back_upper_left[1];
    //      outf << " " << obj->back_lower_right[0];
    //      outf << " " << obj->back_lower_right[1];
    //    }
    outf << " " << obj->car_light.brake_visible;
    outf << " " << obj->car_light.brake_switch_on;
    outf << " " << obj->car_light.left_turn_visible;
    outf << " " << obj->car_light.left_turn_switch_on;
    outf << " " << obj->car_light.right_turn_visible;
    outf << " " << obj->car_light.right_turn_switch_on;

    outf << std::endl;
  }
  outf.close();
  return 0;
}

int WriteDetections(const bool enabled, const std::string &out_path,
                    CameraFrame *frame) {
  if (!enabled) {
    return -1;
  }
  if (frame == nullptr) {
    return -1;
  }
  std::ofstream outf(out_path, std::ofstream::out);
  if (!outf.is_open()) {
    AERROR << "Cannot open output file: " << out_path;
    return -1;
  }
  outf << frame->frame_id << std::endl;
  outf << frame->detected_objects.size() << std::endl;
  const float *feature_ptr = frame->track_feature_blob->cpu_data();
  int feature_dim =
      frame->track_feature_blob->count() / frame->track_feature_blob->num();
  AINFO << "Feature dim: " << feature_dim;
  for (const auto &obj : frame->detected_objects) {
    base::RectF rect(obj->camera_supplement.box);
    outf << " " << rect.x;
    outf << " " << rect.y;
    outf << " " << rect.width;
    outf << " " << rect.height;
    outf << " " << obj->type_probs[static_cast<int>(obj->type)];
    for (int i = 0; i < feature_dim; ++i) {
      outf << " " << *feature_ptr;
      ++feature_ptr;
    }
    outf << std::endl;
  }
  return 0;
}

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

void WriteFusionTracking(std::ofstream &fout, int frame_num,
                         const std::string &camera_name,
                         const std::vector<base::ObjectPtr> &tracked_object) {
  if (!fout.is_open()) {
    AERROR << "Failed to write tracking!";
    return;
  }
  AINFO << "Write track results: " << frame_num;
  if (camera_name == "front_12mm") {
    for (size_t i = 0; i < tracked_object.size(); ++i) {
      base::ObjectPtr ptr = tracked_object[i];
      char output[300];
      snprintf(output, sizeof(output),
               "%d %d %s -1 -1 %2.3f %4.3f %4.3f %4.3f %4.3f "
               "%2.6f %2.6f %2.6f %2.3f %2.3f %2.3f %2.3f %.3f "
               "%2.6f %2.6f %2.6f",
               frame_num, ptr->track_id,
               sub_type_string[static_cast<int>(ptr->sub_type)].c_str(),
               ptr->camera_supplement.alpha, ptr->camera_supplement.box.xmin,
               ptr->camera_supplement.box.ymin, ptr->camera_supplement.box.xmax,
               ptr->camera_supplement.box.ymax, ptr->size[2], ptr->size[1],
               ptr->size[0], ptr->center[0], ptr->center[1], ptr->center[2],
               ptr->theta, ptr->type_probs[static_cast<int>(ptr->type)],
               ptr->velocity[0], ptr->velocity[1], ptr->velocity[2]);
      fout << output << std::endl;
    }
  } else if (camera_name == "front_6mm") {
    for (size_t i = 0; i < tracked_object.size(); ++i) {
      base::ObjectPtr ptr = tracked_object[i];
      char output[300];
      snprintf(output, sizeof(output),
               "%d %d %s -1 -1 %2.3f %4.3f %4.3f %4.3f %4.3f "
               "%2.6f %2.6f %2.6f %2.3f %2.3f %2.3f %2.3f %.3f "
               "%2.6f %2.6f %2.6f",
               frame_num, ptr->track_id,
               sub_type_string[static_cast<int>(ptr->sub_type)].c_str(),
               ptr->camera_supplement.alpha,
               ptr->camera_supplement.projected_box.xmin,
               ptr->camera_supplement.projected_box.ymin,
               ptr->camera_supplement.projected_box.xmax,
               ptr->camera_supplement.projected_box.ymax, ptr->size[2],
               ptr->size[1], ptr->size[0], ptr->center[0], ptr->center[1],
               ptr->center[2], ptr->theta,
               ptr->type_probs[static_cast<int>(ptr->type)], ptr->velocity[0],
               ptr->velocity[1], ptr->velocity[2]);
      fout << output << std::endl;
    }
  } else {
    AERROR << "Unknown camera name: " << camera_name;
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
