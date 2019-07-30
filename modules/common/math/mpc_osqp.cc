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

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {

using Eigen::Sparse;

MpcOsqp::MpcOsqp(
    const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
    const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
    const Eigen::MatrixXd &matrix_lower, const Eigen::MatrixXd &matrix_upper,
    const Eigen::MatrixXd &matrix_initial_state, const int max_iter) {
  matrix_a_ = matrix_a;
  matrix_b_ = matrix_b;
  matrix_q_ = matrix_q;
  matrix_r_ = matrix_r;
  matrix_lower_ = matrix_lower;
  matrix_upper_ = matrix_upper;
  matrix_initial_state_ = matrix_initial_state;
  max_iteration_ = max_iter;
}

void MpcOsqp::castMPCToQPHessian() {
  hessian_.resize(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                  state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);

  // populate hessian matrix
  for (int i = 0; i < state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
       i++) {
    // state range
    if (i < state_dim_ * (horizon_ + 1)) {
      int posQ = i % state_dim_;
      double value = matrix_q_(posQ, posQ);
      if (value != 0) hessian_.insert(i, i) = value;
    } else {
      int posR = i % control_dim_;
      double value = matrix_r_(posR, posR);
      if (value != 0) hessian_.insert(i, i) = value;
    }
  }
}

// error reference is always zero
void MpcOsqp::castMPCToQPGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
}

void MpcOsqp::castMPCToQPConstraintMatrix() {
  matrix_constraint_.resize(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);

  // populate linear constraint matrix
  for (int i = 0; i < state_dim_ * (horizon_ + 1); i++) {
    matrix_constraint_.insert(i, i) = -1;
  }
  // TODO(SHU) change this. low efficient
  for (int i = 0; i < horizon_; i++)
    for (int j = 0; j < state_dim_; j++)
      for (int k = 0; k < state_dim_; k++) {
        double value = matrix_a_(j, k);
        if (value != 0) {
          matrix_constraint_.insert(state_dim_ * (i + 1) + j,
                                    state_dim_ * i + k) = value;
        }
      }
  // TODO(SHU) change this. low efficient
  for (int i = 0; i < horizon_; i++)
    for (int j = 0; j < state_dim_; j++)
      for (int k = 0; k < control_dim_; k++) {
        double value = matrix_b_(j, k);
        if (value != 0) {
          matrix_constraint_.insert(
              state_dim_ * (i + 1) + j,
              control_dim_ * i + k + state_dim_ * (horizon_ + 1)) = value;
        }
      }

  for (int i = 0; i < state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
       i++) {
    matrix_constraint_.insert(i + (horizon_ + 1) * state_dim_, i) = 1;
  }
}

void MpcOsqp::castMPCToQPConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::Matrix<double, 4, 1> xMax =
      Eigen::MatrixXd::Constant(state_dim_, 1, 100.0);
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (int i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = -1.0 * xMax;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = xMax;
  }
  for (int i = 0; i < horizon_; i++) {
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_upper_;
  }

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = matrix_initial_state_;
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;

  // merge inequality and equality vectors
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;

  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
}

OSQPSettings *MpcOsqp::Settings() {
  // default setting
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->scaled_termination = true;
  settings->verbose = false;
  settings->max_iter = max_iteration_;
  settings->warm_start = true;
  return settings;
}

OSQPData *MpcOsqp::Data() {
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  size_t num_affine_constraint =
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  data->n = kernel_dim;
  data->m = num_affine_constraint;
  createOsqpSparseMatrix(hessian_, &(data->P));
  data->q = gradient_.data();
  createOsqpSparseMatrix(matrix_constraint_, &(data->A));
  data->l = lowerBound_.data();
  data->u = upperBound_.data();
  return data;
}

bool MpcOsqp::createOsqpSparseMatrix(
    const Eigen::SparseMatrix<double> &eigenSparseMatrix,
    csc **osqpSparseMatrixPtr) {
  c_int rows = eigenSparseMatrix.rows();
  c_int cols = eigenSparseMatrix.cols();
  c_int numberOfNonZeroCoeff = eigenSparseMatrix.nonZeros();

  const int *outerIndexPtr = eigenSparseMatrix.outerIndexPtr();
  const int *innerNonZerosPtr = eigenSparseMatrix.innerNonZeroPtr();
  csc *osqpSparseMatrix = *(osqpSparseMatrixPtr);

  if (osqpSparseMatrix != nullptr) {
    AERROR << "osqpSparseMatrix pointer is not a null pointer! ";
    return false;
  }

  osqpSparseMatrix = csc_spalloc(rows, cols, numberOfNonZeroCoeff, 1, 0);

  int innerOsqpPosition = 0;
  for (int k = 0; k < cols; k++) {
    if (eigenSparseMatrix.isCompressed()) {
      osqpSparseMatrix->p[k] = static_cast<c_int>(outerIndexPtr[k]);
    } else {
      if (k == 0) {
        osqpSparseMatrix->p[k] = 0;
      } else {
        osqpSparseMatrix->p[k] =
            osqpSparseMatrix->p[k - 1] + innerNonZerosPtr[k - 1];
      }
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(eigenSparseMatrix, k);
         it; ++it) {
      osqpSparseMatrix->i[innerOsqpPosition] = static_cast<c_int>(it.row());
      osqpSparseMatrix->x[innerOsqpPosition] = static_cast<c_float>(it.value());
      innerOsqpPosition++;
    }
  }
  osqpSparseMatrix->p[static_cast<int>(cols)] =
      static_cast<c_int>(innerOsqpPosition);

  CHECK(innerOsqpPosition == numberOfNonZeroCoeff);

  return true;
}
void MpcOsqp::FreeData(OSQPData *data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

bool MpcOsqp::MpcOsqpSolver(std::vector<double> *control_cmd) {
  // cast the MPC problem as QP problem
  castMPCToQPHessian();
  castMPCToQPGradient();
  castMPCToQPConstraintMatrix();
  castMPCToQPConstraintVectors();

  OSQPData *data = Data();
  OSQPSettings *settings = Settings();
  OSQPWorkspace *osqp_workspace = osqp_setup(data, settings);
  osqp_solve(osqp_workspace);
  auto status = osqp_workspace->info->status_val;

  // check status
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }

  //   c_float *solution = osqp_workspace->solution->x;
  int first_control = state_dim_ * (horizon_ + 1);
  for (int i = 0; i < control_dim_; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
  }

  // Cleanup
  osqp_cleanup(osqp_workspace);
  FreeData(data);
  c_free(settings);
  return true;
}  // namespace math
}  // namespace math
}  // namespace common
}  // namespace apollo
