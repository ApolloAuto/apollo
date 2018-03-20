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
#ifndef MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_FUSER_UTIL_H_
#define MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_FUSER_UTIL_H_

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"

#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

enum {
  VALID_OBJECT_TYPE = static_cast<int>(ObjectType::MAX_OBJECT_TYPE) - 2,
};

typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, 1> Vectord;
typedef Eigen::Matrix<int, VALID_OBJECT_TYPE, 1> Vectori;
typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, VALID_OBJECT_TYPE> Matrixd;

namespace fuser_util {

/**
 * @brief Convert probability format from std to eigen vector
 * @param src_prob Input probability stored in std vector
 * @param dst_prob Output probability stored in eigen vector
 */
void FromStdVector(const std::vector<float>& src_prob, Vectord* dst_prob);

/**
 * @brief Convert probability format from eigen to std vector
 * @param src_prob Input probability stored in eigen vector
 * @param dst_prob Output probability stored in std vector
 */
void FromEigenVector(const Vectord& src_prob, std::vector<float>* dst_prob);

/**
 * @brief Transform probability into that in log space
 * @param In and out probability
 */
void ToLog(Vectord* prob);

/**
 * @brief Transform probability into that in exp space
 * @param In and out probability
 */
void ToExp(Vectord* prob);

/**
 * @brief Normalize the probability
 * @param In and out probability
 */
void Normalize(Vectord* prob);

/**
 * @brief Normalize the probability stored as a row vector in eigen matrix
 * @param In and out probability
 */
void NormalizeRow(Matrixd* prob);

/**
 * @brief Print probability
 * @param prob Probability to be printed
 * @param name Name of probability to be printed
*/
void PrintProbability(const std::vector<float>& prob, const std::string& name);

/**
 * @brief Load a matrix from input file stream
 * @param fin The input file stream
 * @param matrix The loaded Matrix
 * @return True if load successfully, false otherwise
 */
bool LoadSingleMatrix(std::ifstream& fin, Matrixd* matrix);

/**
 * @brief Load a matrix from file
 * @param filename The file name
 * @param matrix The loaded Matrix
 * @return True if load successfully, false otherwise
 */
bool LoadSingleMatrixFile(const std::string& filename, Matrixd* matrix);

/**
 * @brief Load multiple matrices from file
 * @param filename The file name
 * @param matrices The loaded Matrices
 * @return True if load successfully, false otherwise
 */
bool LoadMultipleMatricesFile(
    const std::string& filename,
    std::unordered_map<std::string, Matrixd>* matrices);

}  // namespace fuser_util
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_FUSER_UTIL_H_
