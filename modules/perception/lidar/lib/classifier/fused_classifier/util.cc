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
#include "modules/perception/lidar/lib/classifier/fused_classifier/util.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lidar {
namespace util {

using apollo::perception::base::ObjectType;

void FromStdToVector(const std::vector<float>& src_prob, Vectord* dst_prob) {
  (*dst_prob)(0) = src_prob[0];
  for (size_t i = 3; i < static_cast<size_t>(ObjectType::MAX_OBJECT_TYPE);
       ++i) {
    (*dst_prob)(i - 2) = static_cast<double>(src_prob[i]);
  }
}

void FromEigenToVector(const Vectord& src_prob, std::vector<float>* dst_prob) {
  dst_prob->assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  dst_prob->at(0) = static_cast<float>(src_prob(0));
  for (size_t i = 3; i < static_cast<size_t>(ObjectType::MAX_OBJECT_TYPE);
       ++i) {
    dst_prob->at(i) = static_cast<float>(src_prob(i - 2));
  }
}

void ToLog(Vectord* prob) {
  for (size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    (*prob)(i) = log((*prob)(i));
  }
}

void ToExp(Vectord* prob) {
  for (size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    (*prob)(i) = exp((*prob)(i));
  }
}

void ToExpStable(Vectord* prob) {
  double min_value = prob->minCoeff();
  for (size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
    (*prob)(i) = exp((*prob)(i)-min_value);
  }
}

void Normalize(Vectord* prob) {
  double sum = prob->sum();
  sum = sum < 1e-9 ? 1e-9 : sum;
  *prob /= sum;
}

void NormalizeRow(Matrixd* prob) {
  double sum = 0.0;
  for (size_t row = 0; row < VALID_OBJECT_TYPE; ++row) {
    sum = 0.0;
    for (size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
      sum += (*prob)(row, col);
    }
    sum = sum < 1e-9 ? 1e-9 : sum;
    for (size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
      (*prob)(row, col) /= sum;
    }
  }
}

bool LoadSingleMatrix(std::ifstream& fin, Matrixd* matrix) {
  for (size_t row = 0; row < VALID_OBJECT_TYPE; ++row) {
    for (size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
      fin >> (*matrix)(row, col);
    }
  }
  return true;
}

bool LoadSingleMatrixFile(const std::string& filename, Matrixd* matrix) {
  if (matrix == nullptr) {
    return false;
  }
  std::ifstream fin(filename);
  if (!fin.is_open()) {
    AERROR << "Fail to open file: " << filename;
    return false;
  }
  LoadSingleMatrix(fin, matrix);
  fin.close();
  return true;
}

bool LoadMultipleMatricesFile(const std::string& filename,
                              std::map<std::string, Matrixd>* matrices) {
  if (matrices == nullptr) {
    return false;
  }
  std::ifstream fin(filename);
  if (!fin.is_open()) {
    AERROR << "Fail to open file: " << filename;
    return false;
  }
  matrices->clear();
  size_t num = 0;
  fin >> num;
  for (size_t i = 0; i < num; ++i) {
    std::string name;
    fin >> name;
    Matrixd matrix;
    LoadSingleMatrix(fin, &matrix);
    matrices->emplace(name, matrix);
  }
  fin.close();
  return true;
}

}  // namespace util
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
