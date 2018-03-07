/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/camera/common/projector.h"

#include <fstream>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::vector;
using std::string;
using std::ifstream;
using Eigen::MatrixXf;
using Eigen::Map;

bool MatrixProjector::project(vector<float>* feature) {
  if (feature == nullptr) {
    AERROR << "feature is a null pointer.";
    return false;
  }

  if (feature->size() > 0) {
    feature->insert(feature->begin(), 1.0f);
    MatrixXf v = Map<MatrixXf>(&(feature->at(0)), 1, feature->size());
    MatrixXf projected_feature = v * matrix_;

    feature->resize(projected_feature.size());
    float *project_data = projected_feature.data();
    for (auto feat = feature->begin(); feat != feature->end(); ++feat) {
      *feat = *project_data++;
    }
  }

  return true;
}

MatrixProjector::MatrixProjector(string weight_file) {
  ifstream f_in(weight_file.c_str());
  int height = 0;
  int width = 0;
  char comment = ' ';
  float temp = 0;
  f_in >> comment >> height >> width;
  vector<float> theta(height * width);
  for (auto i = theta.begin(); i != theta.end(); ++i) {
    f_in >> temp;
    *i = temp;
  }
  matrix_ = Map<MatrixXf>(&theta[0], width, height);
  matrix_.transposeInPlace();
}

}  // namespace perception
}  // namespace apollo
