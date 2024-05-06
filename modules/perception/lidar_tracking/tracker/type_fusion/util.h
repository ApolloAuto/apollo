/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace lidar {

enum {
    VALID_OBJECT_TYPE = static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE) - 2
};

typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, 1> Vectord;
typedef Eigen::Matrix<int, VALID_OBJECT_TYPE, 1> Vectori;
typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, VALID_OBJECT_TYPE> Matrixd;

namespace type_util {

/**
 * @brief From std to vector
 *
 * @param src_prob
 * @param dst_prob
 */
void FromStdToVector(const std::vector<float>& src_prob, Vectord* dst_prob);

/**
 * @brief Transfrom Eigen to vector
 *
 * @param src_prob
 * @param dst_prob
 */
void FromEigenToVector(const Vectord& src_prob, std::vector<float>* dst_prob);

/**
 * @brief Compute log of Vectord
 *
 * @param prob
 */
void ToLog(Vectord* prob);

/**
 * @brief Compute exponential of Vectord
 *
 * @param prob
 */
void ToExp(Vectord* prob);

/**
 * @brief Compute stable exponential of Vectord
 *
 * @param prob
 */
void ToExpStable(Vectord* prob);

/**
 * @brief Compute normalize of Vectord
 *
 * @param prob
 */
void Normalize(Vectord* prob);

/**
 * @brief Compute normalize row of Matrixd
 *
 * @param prob
 */
void NormalizeRow(Matrixd* prob);

/**
 * @brief Load single matrix
 *
 * @param fin
 * @param matrix
 * @return true
 * @return false
 */
bool LoadSingleMatrix(std::ifstream& fin, Matrixd* matrix);

/**
 * @brief Load single matrix from file
 *
 * @param filename file to load matrix
 * @param matrix
 * @return true
 * @return false
 */
bool LoadSingleMatrixFile(const std::string& filename, Matrixd* matrix);

/**
 * @brief Load multiple matrices from file
 *
 * @param filename file to load matrices
 * @param matrices
 * @return true
 * @return false
 */
bool LoadMultipleMatricesFile(const std::string& filename,
    apollo::common::EigenMap<std::string, Matrixd>* matrices);

}  // namespace type_util
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
