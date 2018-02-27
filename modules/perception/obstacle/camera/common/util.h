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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>


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

class Timer {
 public:
  Timer() {
    _scale = 1.0 / (static_cast<double>(cvGetTickFrequency()) * 1000.);
    tic();
  }
  void tic() {
    _start = static_cast<double>(cv::getTickCount());
  }
  double toc(bool reset = false) {
    double time = (static_cast<double>(cvGetTickCount()) - _start) * _scale;
    if (reset) {
      tic();
    }
    return time;
  }

 private:
  double _start;
  double _scale;
};

inline void l2norm(float *feat_data, int feat_dim) {
  if (feat_dim == 0) {
    return;
  }
  // feature normalization
  float l2norm = 0.0;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.0 / sqrt(feat_dim);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

bool load_visual_object_form_file(const std::string &file_name,
                                  std::vector<VisualObjectPtr> *visual_objects);

bool write_visual_object_to_file(const std::string &file_name,
                                 std::vector<VisualObjectPtr> *visual_objects);

bool load_gt_form_file(const std::string &gt_path,
                       std::vector<VisualObjectPtr> *visual_objects);

std::string get_type_text(ObjectType type);

ObjectType get_object_type(const std::string &type_text);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_UTIL_H
