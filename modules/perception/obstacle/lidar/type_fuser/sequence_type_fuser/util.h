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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_UTIL_H_
#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

enum {
    VALID_OBJECT_TYPE = MAX_OBJECT_TYPE - 2,
};

typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, 1> Vectord;
typedef Eigen::Matrix<int, VALID_OBJECT_TYPE, 1> Vectori;
typedef Eigen::Matrix<double, VALID_OBJECT_TYPE, VALID_OBJECT_TYPE> Matrixd;

namespace sequence_type_fuser {

void from_std_vector(const std::vector<float>& src_prob, Vectord* dst_prob);

void from_eigen_vector(const Vectord& src_prob, std::vector<float>* dst_prob);

void to_log(Vectord* prob);

// void to_exp(Vectord* prob);

void to_exp(Vectord* prob);

void normalize(Vectord* prob);

// void normalize_log_space(Vectord* prob);

void normalize_row(Matrixd* prob);

void print_probability(const std::vector<float>& prob, const std::string& name);

bool load_single_matrix(std::ifstream& fin, Matrixd* matrix);

bool load_single_matrix_file(const std::string& filename, Matrixd* matrix);

bool load_multiple_matrices_file(const std::string& filename, 
        std::map<std::string, Matrixd>* matrices);

}  // namespace sequence_type_fuser
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEQUENCE_TYPE_FUSER_UTIL_H_
