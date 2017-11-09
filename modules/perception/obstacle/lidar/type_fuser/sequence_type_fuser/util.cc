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
#include "modules/perception/obstacle/lidar/type_fuser/sequence_type_fuser/util.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace sequence_type_fuser {

void from_std_vector(const std::vector<float>& src_prob, 
        Vectord* dst_prob) {
    (*dst_prob)(0) = src_prob[0];
    for (std::size_t i = 3; i < MAX_OBJECT_TYPE; ++i) {
        (*dst_prob)(i-2) = static_cast<double>(src_prob[i]);
    }
}

void from_eigen_vector(const Vectord& src_prob, 
        std::vector<float>* dst_prob) {
    dst_prob->assign(MAX_OBJECT_TYPE, 0);
    dst_prob->at(0) = src_prob(0);
    for (std::size_t i = 3; i < MAX_OBJECT_TYPE; ++i) {
        dst_prob->at(i) = static_cast<float>(src_prob(i-2));
    }
}

void to_log(Vectord* prob) {
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        (*prob)(i) = log((*prob)(i));
    }
}

void to_exp(Vectord* prob) {
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        (*prob)(i) = exp((*prob)(i));
    }
}

void to_exp_stable(Vectord* prob) {
    double min_value = prob->minCoeff();
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        (*prob)(i) = exp((*prob)(i) - min_value);
    }
}

void normalize(Vectord* prob) {
    double sum = prob->sum();
    sum = sum < 1e-9 ? 1e-9 : sum;
    *prob /= sum;
}

void normalize_log_space(Vectord* prob) {
    double sum = 0.0;
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        sum += exp((*prob)(i));
    }
    sum = log(sum);
    for (std::size_t i = 0; i < VALID_OBJECT_TYPE; ++i) {
        (*prob)(i) -= sum;
    }
}

void normalize_row(Matrixd* prob) {
    double sum = 0.0;
    for (std::size_t row = 0; row < VALID_OBJECT_TYPE; ++row) {
        sum = 0.0;
        for (std::size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
            sum += (*prob)(row, col);
        }
        sum = sum < 1e-9 ? 1e-9 : sum;
        for (std::size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
            (*prob)(row, col) /= sum;
        }
    }
}

void print_probability(const std::vector<float>& prob, const std::string& name) {
    std::cout << name << ": ";
    float max_prob = -DBL_MAX;
    std::size_t max_id = 0;
    for (std::size_t i = 0; i < prob.size(); ++i) {
        std::cout << std::setprecision(3) << prob[i] << " ";
        if (prob[i] > max_prob) {
            max_prob = prob[i];
            max_id = i;
        }
    }
    std::cout << " max_type: " << max_id << std::endl;
}

bool load_single_matrix(std::ifstream& fin, Matrixd* matrix) {
    for (std::size_t row = 0; row < VALID_OBJECT_TYPE; ++row) {
        for (std::size_t col = 0; col < VALID_OBJECT_TYPE; ++col) {
            fin >> (*matrix)(row, col);
        }
    }
    return true;
}

bool load_single_matrix_file(const std::string& filename, Matrixd* matrix) {
    if (matrix == nullptr) {
        return false;
    }
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        AERROR << "Fail to open file: " << filename;
        return false;
    }
    load_single_matrix(fin, matrix);
    fin.close();
    return true;
}

bool load_multiple_matrices_file(const std::string& filename, 
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
    std::size_t num = 0;
    fin >> num;
    for (std::size_t i = 0; i < num; ++i) {
        std::string name;
        fin >> name;
        Matrixd matrix;
        load_single_matrix(fin, &matrix);
        matrices->emplace(name, matrix);
    }
    fin.close();
    return true;
}

}  // namsepace sequence_type_fuser
}  // namespace perception
}  // namespace apollo

