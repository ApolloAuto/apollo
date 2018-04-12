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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H_

#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/gzip_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "opencv2/opencv.hpp"

#include "modules/perception/obstacle/camera/common/visual_object.h"

namespace apollo {
namespace perception {

extern std::vector<cv::Scalar> color_table;

// Color definition for CV
const cv::Scalar COLOR_WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar COLOR_GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar COLOR_BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar COLOR_YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar COLOR_RED = cv::Scalar(0, 0, 255);
const cv::Scalar COLOR_BLACK = cv::Scalar(0, 0, 0);

bool LoadVisualObjectFromFile(
    const std::string &file_name,
    std::vector<std::shared_ptr<VisualObject>> *visual_objects);

bool WriteVisualObjectToFile(
    const std::string &file_name,
    std::vector<std::shared_ptr<VisualObject>> *visual_objects);

bool LoadGTfromFile(const std::string &gt_path,
                    std::vector<std::shared_ptr<VisualObject>> *visual_objects);

std::string GetTypeText(ObjectType type);

ObjectType GetObjectType(const std::string &type_text);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H_
