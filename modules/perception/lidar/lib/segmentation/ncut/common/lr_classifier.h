/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <math.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

class LRClassifier {
 public:
  LRClassifier() = default;
  ~LRClassifier() {}
  bool init() {
    _lr_parameters.resize(3, 4);
    _lr_parameters.coeffRef(0, 0) = 0.0510903f;
    _lr_parameters.coeffRef(0, 1) = -1.00989f;
    _lr_parameters.coeffRef(0, 2) = -1.6537f;
    _lr_parameters.coeffRef(0, 3) = 0.130055f;
    _lr_parameters.coeffRef(1, 0) = 0.266469f;
    _lr_parameters.coeffRef(1, 1) = -0.538964f;
    _lr_parameters.coeffRef(1, 2) = -0.291611f;
    _lr_parameters.coeffRef(1, 3) = -0.070701f;
    _lr_parameters.coeffRef(2, 0) = 0.497949f;
    _lr_parameters.coeffRef(2, 1) = -0.504843f;
    _lr_parameters.coeffRef(2, 2) = -0.152141f;
    _lr_parameters.coeffRef(2, 3) = -1.38024f;
    return true;
  }

  std::string GetLabel(base::PointFCloudConstPtr cloud) {
    // point cloud should be rotated
    float x_max = -FLT_MAX;
    float y_max = -FLT_MAX;
    float z_max = -FLT_MAX;
    float x_min = FLT_MAX;
    float y_min = FLT_MAX;
    float z_min = FLT_MAX;
    for (size_t i = 0; i < cloud->size(); ++i) {
      auto pt = (*cloud)[i];
      x_min = std::min(x_min, pt.x);
      x_max = std::max(x_max, pt.x);
      y_min = std::min(y_min, pt.y);
      y_max = std::max(y_max, pt.y);
      z_min = std::min(z_min, pt.z);
      z_max = std::max(z_max, pt.z);
    }
    Eigen::Vector3f fea = {x_max - x_min, y_max - y_min, z_max - z_min};
    Eigen::VectorXf response = fea.transpose() * _lr_parameters;
    int type = 0;
    // float max_score = response.maxCoeff(&type);
    return _labels[type];
  }

 private:
  Eigen::MatrixXf _lr_parameters;
  std::vector<std::string> _labels = {"unknown", "nonMot", "pedestrian",
                                      "smallMot"};
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
