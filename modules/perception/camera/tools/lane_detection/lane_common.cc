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
#include "modules/perception/camera/tools/lane_detection/lane_common.h"

#include "cyber/common/log.h"

DEFINE_string(list, "test.list", "test file title");
DEFINE_string(file_title, "", "test file title");
DEFINE_string(debug_file, "", "debug file title");
DEFINE_string(save_dir, "./result/", "test file title");
DEFINE_string(file_ext_name, "", "file extension name");
DEFINE_string(file_debug_list, "", "file extension name");
DEFINE_bool(lane_line_debug, false, "draw the lane line result");
DEFINE_bool(lane_cc_debug, false, "show lane cc image");
DEFINE_bool(lane_center_debug, false, "draw the lane center result");
DEFINE_bool(lane_ego_debug, false, "lane ego debug");
DEFINE_bool(lane_result_output, false, "output the lane result");
DEFINE_bool(lane_points_output, false, "output the detected lane points");
DEFINE_string(image_dir, "./image/", "test image directory");
DEFINE_string(camera_intrinsics_yaml, "params/front_6mm_intrinsics.yaml",
              "camera intrinsics_yaml");

namespace apollo {
namespace perception {
namespace camera {
// show detect point_set
void show_detect_point_set(
    const cv::Mat& image,
    const std::vector<std::vector<LanePointInfo> >& detect_laneline_point_set,
    const std::string& save_path) {
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;

  cv::Mat draw_mat = image.clone();
  for (size_t line_idx = 0; line_idx < detect_laneline_point_set.size();
       ++line_idx) {
    if (line_idx == 0) {
      color = cv::Scalar(0, 255, 0);
    } else if (line_idx == 1) {
      color = cv::Scalar(255, 0, 0);
    } else if (line_idx == 2) {
      color = cv::Scalar(255, 255, 0);
    } else if (line_idx == 3) {
      color = cv::Scalar(0, 255, 255);
    }
    for (size_t i = 0; i < detect_laneline_point_set[line_idx].size(); ++i) {
      int point_x = static_cast<int>(detect_laneline_point_set[line_idx][i].x);
      int point_y = static_cast<int>(detect_laneline_point_set[line_idx][i].y);
      cv::Point draw_point(point_x, point_y);
      cv::circle(draw_mat, draw_point, draw_size, color, 4);
    }
  }
  cv::imwrite(save_path, draw_mat);
}

void show_all_infer_point_set(const cv::Mat& image,
                              const std::vector<LanePointInfo>& infer_point_set,
                              const std::string& save_path) {
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;

  cv::Mat draw_mat = image.clone();
  for (size_t i = 0; i < infer_point_set.size(); ++i) {
    int point_x = static_cast<int>(infer_point_set[i].x);
    int point_y = static_cast<int>(infer_point_set[i].y);
    cv::circle(draw_mat, cv::Point(point_x, point_y), draw_size, color, 4);
  }
  cv::imwrite(save_path, draw_mat);
}

void show_lane_lines(const cv::Mat& image,
                     const std::vector<base::LaneLine>& lane_marks,
                     const std::string& save_path) {
  const float range_x = 70.0f;
  const float range_y = 30.0f;
  const float pixels_per_meter = 7.0f;
  const int ipm_height = static_cast<int>(range_x * pixels_per_meter);
  const int ipm_width = static_cast<int>(range_y * pixels_per_meter);
  cv::Mat draw_ipm = cv::Mat::zeros(ipm_height, ipm_width, CV_8UC3);
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;
  cv::Mat draw_mat = image.clone();
  for (size_t i = 0; i < lane_marks.size(); ++i) {
    base::LaneLinePositionType pos_type = lane_marks[i].pos_type;
    if (pos_type == base::LaneLinePositionType::EGO_LEFT ||
        pos_type == base::LaneLinePositionType::EGO_RIGHT) {
      color = cv::Scalar(0, 255, 0);
    } else if (pos_type == base::LaneLinePositionType::ADJACENT_LEFT ||
               pos_type == base::LaneLinePositionType::ADJACENT_RIGHT) {
      color = cv::Scalar(0, 0, 255);
    } else {
      color = cv::Scalar(255, 255, 0);
    }
    // Draw image curve
    float fa = lane_marks[i].curve_image_coord.a;
    float fb = lane_marks[i].curve_image_coord.b;
    float fc = lane_marks[i].curve_image_coord.c;
    float fd = lane_marks[i].curve_image_coord.d;
    float y0 = lane_marks[i].curve_image_coord.x_start;
    float y1 = lane_marks[i].curve_image_coord.x_end;
    for (int j = static_cast<int>(y0); j <= static_cast<int>(y1); ++j) {
      int x = static_cast<int>(fa * pow(j, 3) + fb * pow(j, 2) +
                               fc * static_cast<float>(j) + fd);
      cv::circle(draw_mat, cv::Point(x, j), draw_size, color);
    }
    // Draw ipm curve
    float camera_xs = lane_marks[i].curve_camera_coord.x_start;
    float camera_xe = lane_marks[i].curve_camera_coord.x_end;
    float camera_fa = lane_marks[i].curve_camera_coord.a;
    float camera_fb = lane_marks[i].curve_camera_coord.b;
    float camera_fc = lane_marks[i].curve_camera_coord.c;
    float camera_fd = lane_marks[i].curve_camera_coord.d;
    for (float j = camera_xs; j <= camera_xe; j += 0.1f) {
      float y =
          static_cast<float>(pow(j, 3) * camera_fa + pow(j, 2) * camera_fb +
                             static_cast<float>(j) * camera_fc + camera_fd);
      int draw_y = ipm_height - static_cast<int>(j * pixels_per_meter);
      int draw_x = ipm_width / 2 + static_cast<int>(y * pixels_per_meter);
      if (draw_x < 0 || draw_x >= ipm_width) {
        continue;
      }
      if (draw_y < 0 || draw_y >= ipm_height) {
        continue;
      }
      cv::circle(draw_ipm, cv::Point(draw_x, draw_y), 2, color);
    }
  }
  cv::circle(draw_ipm, cv::Point(ipm_width / 2, ipm_height - 2), 5,
             cv::Scalar(0, 0, 255), 5);
  cv::line(draw_ipm, cv::Point(ipm_width / 2, 0),
           cv::Point(ipm_width / 2, ipm_height), cv::Scalar(255, 255, 255));
  for (int i = 10; i < static_cast<int>(range_x); i += 10) {
    int x = ipm_width / 2;
    int y =
        ipm_height - static_cast<int>(static_cast<float>(i) * pixels_per_meter);
    cv::line(draw_ipm, cv::Point(0, y), cv::Point(ipm_width, y),
             cv::Scalar(255, 255, 255));
    std::string label = cv::format("%d", i);
    cv::putText(draw_ipm, label, cv::Point(x, y), CV_FONT_HERSHEY_COMPLEX_SMALL,
                1.0, cv::Scalar(255, 255, 255));
  }
  cv::Rect roi(0, 0, ipm_width, ipm_height);
  draw_ipm.copyTo(draw_mat(roi));
  cv::imwrite(save_path, draw_mat);
}

void show_lane_ccs(const std::vector<unsigned char>& lane_map,
                   const int lane_map_width, const int lane_map_height,
                   const std::vector<ConnectedComponent>& lane_ccs,
                   const std::vector<ConnectedComponent>& select_lane_ccs,
                   const std::string& save_path) {
  cv::Mat lane_map_draw =
      cv::Mat::zeros(lane_map_height, lane_map_width, CV_8UC1);
  for (int y = 0; y < lane_map_height; ++y) {
    for (int x = 0; x < lane_map_width; x++) {
      lane_map_draw.at<unsigned char>(y, x) = static_cast<unsigned char>(
          255 - lane_map[y * lane_map_width + x] * 255);
    }
  }
  cv::Mat lane_draw;
  cv::cvtColor(lane_map_draw, lane_draw, 8);
  cv::Scalar color;
  std::string msg = "";
  for (size_t i = 0; i < lane_ccs.size(); ++i) {
    std::vector<base::Point2DI> pixels = lane_ccs[i].GetPixels();
    base::BBox2DI bbox1 = lane_ccs[i].GetBBox();
    int find_index = -1;
    for (size_t j = 0; j < select_lane_ccs.size(); ++j) {
      base::BBox2DI bbox2 = select_lane_ccs[j].GetBBox();
      if (bbox1.xmin == bbox2.xmin && bbox1.xmax == bbox2.xmax &&
          bbox1.ymin == bbox2.ymin && bbox1.ymax == bbox2.ymax) {
        find_index = static_cast<int>(j);
        break;
      }
    }
    if (find_index == 0) {
      color = cv::Scalar(255, 0, 0);
    } else if (find_index == 1) {
      color = cv::Scalar(0, 255, 0);
    } else if (find_index == 2) {
      color = cv::Scalar(0, 0, 255);
    } else {
      color = cv::Scalar(0, 0, 0);
    }
    for (size_t j = 0; j < pixels.size(); ++j) {
      int x = pixels[j].x;
      int y = pixels[j].y;
      cv::circle(lane_draw, cv::Point(x, y), 1, color);
    }
    msg += cv::format("[%d %d] ", i, pixels.size());
  }
  cv::imwrite(save_path, lane_draw);
  AINFO << msg;
}

void output_laneline_to_json(const std::vector<base::LaneLine>& lane_objects,
                             const std::string& save_path) {
  FILE* file_save = fopen(save_path.c_str(), "wt");
  if (!file_save) {
    AERROR << "Failed to open file: " << save_path;
    return;
  }

  int lane_line_size = static_cast<int>(lane_objects.size());
  AINFO << "lane line num: " << lane_line_size;
  std::string msg = "lane line info: ";
  fprintf(file_save, "[\n");
  for (int j = 0; j < lane_line_size; ++j) {
    const base::LaneLineCubicCurve& curve_camera =
        lane_objects[j].curve_camera_coord;
    const base::LaneLineCubicCurve& curve_img =
        lane_objects[j].curve_image_coord;
    const std::vector<base::Point3DF>& camera_point_set =
        lane_objects[j].curve_camera_point_set;
    const std::vector<base::Point2DF>& image_point_set =
        lane_objects[j].curve_image_point_set;
    fprintf(file_save, "{\n");
    fprintf(file_save, "\"type\":%d,\n", lane_objects[j].type);
    fprintf(file_save, "\"pos_type\":%d,\n", lane_objects[j].pos_type);
    //  camera curve
    fprintf(file_save, "\"camera_curve\":\n");
    fprintf(file_save, "{\"a\":%.10f,\"b\":%.10f,\"c\":%.10f,\"d\":%.10f,",
            curve_camera.a, curve_camera.b, curve_camera.c, curve_camera.d);
    fprintf(file_save, "\"x0\":%.10f,\"x1\":%.10f},\n", curve_camera.x_start,
            curve_camera.x_end);
    // camera_image_point_set
    fprintf(file_save, "\"camera_point_set\":\n");
    fprintf(file_save, "[");
    for (size_t k = 0; k < camera_point_set.size(); ++k) {
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
    //  image curve
    fprintf(file_save, "\"image_curve\":\n");
    fprintf(file_save, "{\"a\":%.10f,\"b\":%.10f,\"c\":%.10f,\"d\":%.10f,",
            curve_img.a, curve_img.b, curve_img.c, curve_img.d);
    fprintf(file_save, "\"x0\":%.10f,\"x1\":%.10f},\n", curve_img.x_start,
            curve_img.x_end);
    //  curve_image_point_set
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
}

void output_laneline_to_txt(const std::vector<base::LaneLine>& lane_objects,
                            const std::string& save_path) {
  FILE* file_save = fopen(save_path.c_str(), "wt");
  if (!file_save) {
    AERROR << "Failed to open file: " << save_path;
    return;
  }
  int lane_line_size = static_cast<int>(lane_objects.size());
  AINFO << "lane line num: " << lane_line_size;
  fprintf(file_save, "lane_line_num=%d\n", lane_line_size);
  for (int j = 0; j < lane_line_size; ++j) {
    base::LaneLineCubicCurve curve_camera = lane_objects[j].curve_camera_coord;
    base::LaneLineCubicCurve curve_img = lane_objects[j].curve_image_coord;
    fprintf(file_save, "type=%d\n", lane_objects[j].type);
    fprintf(file_save, "pos_type=%d\n", lane_objects[j].pos_type);
    fprintf(file_save, "camera_curve:\n");
    fprintf(file_save, "a=%f b=%f c=%f d=%f x0=%f x1=%f\n", curve_camera.a,
            curve_camera.b, curve_camera.c, curve_camera.d,
            curve_camera.x_start, curve_camera.x_end);
    fprintf(file_save, "image_curve:\n");
    fprintf(file_save, "a=%f b=%f c=%f d=%f x0=%f x1=%f\n", curve_img.a,
            curve_img.b, curve_img.c, curve_img.d, curve_img.x_start,
            curve_img.x_end);
    fprintf(file_save, "confidence=%f\n", lane_objects[j].confidence);
    std::vector<base::Point2DF> curve_image_point_set =
        lane_objects[j].curve_image_point_set;
    std::vector<base::Point3DF> curve_camera_point_set =
        lane_objects[j].curve_camera_point_set;
    fprintf(file_save, "curve_image_point_set:\n");
    for (size_t k = 0; k < curve_image_point_set.size(); ++k) {
      fprintf(file_save, "[%f %f] ", curve_image_point_set[k].x,
              curve_image_point_set[k].y);
    }
    fprintf(file_save, "\n");
    fprintf(file_save, "curve_camera_point_set:\n");
    for (size_t k = 0; k < curve_camera_point_set.size(); ++k) {
      fprintf(file_save, "[%f %f %f] ", curve_camera_point_set[k].x,
              curve_camera_point_set[k].y, curve_camera_point_set[k].z);
    }
    fprintf(file_save, "\n");
  }
  fclose(file_save);
}

void show_detect_point_set(
    const cv::Mat& image,
    const std::vector<base::Point2DF>& img_laneline_point_set,
    const std::string& save_path) {
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;

  cv::Mat draw_mat = image.clone();
  for (size_t i = 0; i < img_laneline_point_set.size(); ++i) {
    const base::Point2DF& point = img_laneline_point_set[i];
    cv::circle(draw_mat,
               cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
               draw_size, color, 4);
  }
  cv::imwrite(save_path, draw_mat);
}

void show_neighbor_point_set(
    const cv::Mat& image,
    const std::vector<base::Point2DF>& img_laneline_point_set,
    const std::vector<int>& neighbor_point_info, const std::string& save_path) {
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;

  cv::Mat draw_mat = image.clone();
  for (size_t i = 0; i < img_laneline_point_set.size(); ++i) {
    const base::Point2DF& point = img_laneline_point_set[i];
    cv::circle(draw_mat,
               cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
               draw_size, color, 4);
    int neighbor_point_index = neighbor_point_info[i * 2];
    int pass_point_num = -1;
    if (neighbor_point_index != -1) {
      pass_point_num = neighbor_point_info[i * 2 + 1];
      const base::Point2DF& neighbor_point =
          img_laneline_point_set[neighbor_point_index];
      cv::line(draw_mat,
               cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
               cv::Point(static_cast<int>(neighbor_point.x),
                         static_cast<int>(neighbor_point.y)),
               color, 2);
    }
    std::string label = cv::format("%d", pass_point_num);
    cv::putText(draw_mat, label,
                cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
                CV_FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255));
  }
  cv::imwrite(save_path, draw_mat);
}

void show_detect_point_set(
    const cv::Mat& image,
    const std::vector<base::Point2DF>& img_laneline_point_set,
    const std::vector<float>& point_score_vec, const std::string& save_path) {
  cv::Scalar color = cv::Scalar(0, 255, 0);
  int draw_size = 2;

  cv::Mat draw_mat = image.clone();
  for (size_t i = 0; i < img_laneline_point_set.size(); ++i) {
    const base::Point2DF& point = img_laneline_point_set[i];
    std::string label = cv::format("%.2f", point_score_vec[i]);
    cv::putText(draw_mat, label,
                cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
                CV_FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255, 255, 255));
    cv::circle(draw_mat,
               cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)),
               draw_size, color, 4);
  }
  cv::imwrite(save_path, draw_mat);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
