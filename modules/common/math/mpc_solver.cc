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
#include "modules/common/math/mpc_solver.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace common {
namespace math {

using Matrix = Eigen::MatrixXd;

// Linear MPC solver, for single actuator
void SolveLinearMPC(const Matrix &matrix_a,
        const Matrix &matrix_b,
        const Matrix &matrix_c,
        const Matrix &matrix_q,
        const Matrix &matrix_r,
        const Matrix &matrix_lower,
        const Matrix &matrix_upper,
        const Matrix &matrix_initial_state,
        const std::vector<Matrix> &reference,
        const double eps,
        const int max_iter,
        std::vector<Matrix> *control) {
    if (matrix_a.rows() != matrix_a.cols()
            || matrix_b.rows() != matrix_a.rows()
            || matrix_lower.rows() != matrix_upper.rows()) {
        AERROR << "One or more matrices have incompatible dimensions. \
            Aborting.\n";
        return;
    }

    // Initialize matrix_k, matrix_m, matrix_t and matrix_v, matrix_qq, matrix_rr, vector of matrix A power
    Matrix matrix_k = Matrix::Zero(matrix_b.rows() * control->size(), matrix_b.cols() * control->size());
    Matrix matrix_m = Matrix::Zero(matrix_b.rows() * control->size(), 1);
    Matrix matrix_t = matrix_m;
    Matrix matrix_v = Matrix::Zero((*control)[0].rows() * control->size(), 1);
    Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
    Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
    Matrix matrix_ll = Matrix::Zero(control->size() * matrix_lower.rows(), 1);
    Matrix matrix_uu = Matrix::Zero(control->size() * matrix_upper.rows(), 1);
    std::vector<Matrix> matrix_a_power(control->size());

    // Compute power of matrix_a
    matrix_a_power[0] = matrix_a;
    for (unsigned int i = 1; i < matrix_a_power.size(); ++i) {
        matrix_a_power[i] = matrix_a * matrix_a_power[i-1];
    }

    // Compute matrix_k
    for (unsigned int r = 0; r < control->size(); ++r) {
        for (unsigned int c = 0; c <= r; ++c) {
            matrix_k.block(r * matrix_b.rows(), c * matrix_b.cols(), matrix_b.rows(), \
                    matrix_b.cols()) = matrix_a_power[r-c] * matrix_b;
        }
    }

    // Compute matrix_m
    matrix_m.block(0, 0, matrix_a.rows(), 1) = matrix_a * matrix_initial_state + matrix_c;
    for (unsigned int i = 1; i < control->size(); ++i) {
        matrix_m.block(i * matrix_a.rows(), 0, matrix_a.rows(), 1) = matrix_a * matrix_m.block((i-1) * matrix_a.rows(), 0, matrix_a.rows(), 1) + matrix_c;
    }

    // compute matrix_t
    for (unsigned int j = 0; j < reference.size(); ++j) {
        matrix_t.block(j * reference[0].size(), 0, reference[0].size(), 1) = reference[j];
    }

    // compute matrix_v
    for (unsigned int j = 0; j < control->size(); ++j) {
      //  matrix_v.block(j * control->size(), 0, control->size(), 1) = control[j];
        matrix_v.block(j * (*control)[0].rows(), 0, (*control)[0].rows(), 1) = (*control)[j];
    }

    // compute matrix_ll, matrix_uu, matrix_qq, matrix_rr together
    for (unsigned int i = 0; i < control->size(); ++i) {
        matrix_ll.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) = matrix_lower;
        matrix_uu.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) = matrix_upper;
        matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), \
                    matrix_q.rows(), matrix_q.rows()) = matrix_q;
        matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), \
                    matrix_r.rows(), matrix_r.rows()) = matrix_r;
    }

    // update matrix_m1, matrix_m2, convert MPC problem to QP problem done
    Matrix matrix_m1 = matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr;
    Matrix matrix_m2 = matrix_k.transpose() * matrix_qq * (matrix_m - matrix_t);

    // Method 2: QP_SMO_Solver
    SolveQPSMO(matrix_m1, matrix_m2, matrix_ll, matrix_uu, eps, max_iter, &matrix_v);
    for (unsigned int i = 0; i < control->size(); ++i) {
        (*control)[i] = matrix_v.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1);
    }
}

void SolveQPSMO (
        const Matrix& matrix_q,
        const Matrix& matrix_b,
        const Matrix& matrix_lower,
        const Matrix& matrix_upper,
        const double& eps,
        const int& max_iter,
        Matrix* matrix_v) {
    // Warning: Skipped the sanity check since this is designed for solely used by mpc_solver, if you want to
    // use it for other purpose, force sanity check at the beginning
    Matrix matrix_df = matrix_q * (* matrix_v) + matrix_b;
    Matrix matrix_qq = matrix_q.inverse();
    for (int iter = 0; iter < max_iter; ++iter) {
            double max_df = 0;
            int best_r = 0;
            for (int r = 0; r < matrix_q.rows(); ++r)
            {
                if ((*matrix_v)(r) <= matrix_lower(r) && matrix_df(r) > 0) {
                    continue;
                }
                else if ((*matrix_v)(r) >= matrix_upper(r) && matrix_df(r) < 0) {
                    continue;
                }
                else if (std::abs(matrix_df(r)) > max_df)
                {
                    best_r = r;
                    max_df = std::abs(matrix_df(r));
                }
            }

            int r = best_r;
            {
                const double old_alpha = (*matrix_v)(r);
                (*matrix_v)(r) = -(matrix_df(r) - matrix_q(r, r) * (*matrix_v)(r))
                    * matrix_qq(r);
                if ((*matrix_v)(r) < matrix_lower(r)) {
                    (*matrix_v)(r) = matrix_lower(r);
                }
                else if ((*matrix_v)(r) > matrix_upper(r)) {
                    (*matrix_v)(r) = matrix_upper(r);
                }
                const double delta = old_alpha - (*matrix_v)(r);

                // Gradient update.
                for (int k = 0; k < matrix_df.rows(); ++k) {
                    matrix_df(k) -= matrix_q(r, k) * delta;
                }
            }

    AERROR << "max_df is: " << max_df << std::endl;
            if (max_df < eps) {
    AERROR << "max_df is less than eps: " << max_df << std::endl;
    AERROR << "iter now is: " << iter << std::endl;
                break;
            }
        }
}
}  // namespace math
}  // namespace common
}  // namespace apollo
