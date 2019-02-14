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
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "modules/perception/base/lane_struct.h"
#include "modules/perception/base/point.h"
#include "modules/perception/camera/lib/lane/common/common_functions.h"

DECLARE_string(list);
DECLARE_string(file_title);
DECLARE_string(debug_file);
DECLARE_string(save_dir);
DECLARE_string(file_ext_name);
DECLARE_string(file_debug_list);
DECLARE_bool(lane_line_debug);
DECLARE_bool(lane_cc_debug);
#if 0
DECLARE_bool(lane_center_debug);
DECLARE_bool(lane_ego_debug);
#endif
DECLARE_bool(lane_result_output);
#if 0
DECLARE_bool(lane_points_output);
DECLARE_string(image_dir);
#endif
DECLARE_string(camera_intrinsics_yaml);

namespace apollo {
namespace perception {
namespace camera {
// draw detected lane point_set
void show_detect_point_set(
    const cv::Mat& image,
    const std::vector<std::vector<LanePointInfo>>& detect_laneline_point_set,
    const std::string& save_path);

void show_all_infer_point_set(const cv::Mat& image,
                              const std::vector<LanePointInfo>& infer_point_set,
                              const std::string& save_path);

//
void show_lane_lines(const cv::Mat& image,
                     const std::vector<base::LaneLine>& lane_marks,
                     const std::string& save_path);

//
void show_lane_ccs(const std::vector<unsigned char>& lane_map,
                   const int lane_map_width, const int lane_map_height,
                   const std::vector<ConnectedComponent>& lane_ccs,
                   const std::vector<ConnectedComponent>& select_lane_ccs,
                   const std::string& save_path);
//
//  save the lane line and points to json format
void output_laneline_to_json(const std::vector<base::LaneLine>& lane_objects,
                             const std::string& save_path);

void output_laneline_to_txt(const std::vector<base::LaneLine>& lane_objects,
                            const std::string& save_path);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
