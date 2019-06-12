
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
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <algorithm>
#include <limits>
#include <vector>

namespace apollo {
namespace localization {
namespace ndt {

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::Align(
    PointCloudSourcePtr output, const Eigen::Matrix4f &guess) {
  // Resize the output dataset
  if (output->points.size() != input_->size())
    output->points.resize(input_->size());
  // Copy the header
  output->header = input_->header;

  if (input_->width == 1 || input_->height == 1) {
    output->width = static_cast<uint32_t>(input_->size());
    output->height = 1;
  } else {
    output->width = static_cast<uint32_t>(input_->width);
    output->height = input_->height;
  }
  output->is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < input_->size(); ++i) {
    output->points[i] = input_->points[i];
  }

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = Eigen::Matrix4f::Identity();
  transformation_ = Eigen::Matrix4f::Identity();
  previous_transformation_ = Eigen::Matrix4f::Identity();

  // Right before we estimate the transformation, we set all the point.data[3]
  // values to 1 to aid the rigid transformation
  for (size_t i = 0; i < input_->size(); ++i) {
    output->points[i].data[3] = 1.0;
  }

  ComputeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
double NormalDistributionsTransform<PointSource, PointTarget>::GetFitnessScore(
    double max_range) {
  // Set the target tree
  target_tree_->setInputCloud(target_);
  double fitness_score = 0.0;
  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  // transformPointCloud (*input_, input_transformed, final_transformation_);
  input_transformed.resize(input_->size());
  for (size_t i = 0; i < input_->size(); ++i) {
    const PointSource &src = input_->points[i];
    PointTarget &tgt = input_transformed.points[i];
    tgt.x = static_cast<float>(final_transformation_(0, 0) * src.x +
                               final_transformation_(0, 1) * src.y +
                               final_transformation_(0, 2) * src.z +
                               final_transformation_(0, 3));
    tgt.y = static_cast<float>(final_transformation_(1, 0) * src.x +
                               final_transformation_(1, 1) * src.y +
                               final_transformation_(1, 2) * src.z +
                               final_transformation_(1, 3));
    tgt.z = static_cast<float>(final_transformation_(2, 0) * src.x +
                               final_transformation_(2, 1) * src.y +
                               final_transformation_(2, 2) * src.z +
                               final_transformation_(2, 3));
  }

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size(); ++i) {
    Eigen::Vector4f p1 = Eigen::Vector4f(input_transformed.points[i].x,
                                         input_transformed.points[i].y,
                                         input_transformed.points[i].z, 0);
    // Find its nearest neighbor in the target
    target_tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices,
                                 nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range) continue;

    Eigen::Vector4f p2 = Eigen::Vector4f(target_->points[nn_indices[0]].x,
                                         target_->points[nn_indices[0]].y,
                                         target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs((p1 - p2).squaredNorm());
    nr++;
  }

  if (nr > 0) {
    return (fitness_score / nr);
  } else {
    return (std::numeric_limits<double>::max());
  }
}

template <typename PointSource, typename PointTarget>
NormalDistributionsTransform<PointSource,
                             PointTarget>::NormalDistributionsTransform()
    : target_tree_(new KdTree),
      target_cells_(),
      resolution_(1.0f),
      step_size_(0.1),
      outlier_ratio_(0.55),
      gauss_d1_(),
      gauss_d2_(),
      trans_probability_(),
      j_ang_a_(),
      j_ang_b_(),
      j_ang_c_(),
      j_ang_d_(),
      j_ang_e_(),
      j_ang_f_(),
      j_ang_g_(),
      j_ang_h_(),
      h_ang_a2_(),
      h_ang_a3_(),
      h_ang_b2_(),
      h_ang_b3_(),
      h_ang_c2_(),
      h_ang_c3_(),
      h_ang_d1_(),
      h_ang_d2_(),
      h_ang_d3_(),
      h_ang_e1_(),
      h_ang_e2_(),
      h_ang_e3_(),
      h_ang_f1_(),
      h_ang_f2_(),
      h_ang_f3_(),
      point_gradient_(),
      point_hessian_() {
  double gauss_c1, gauss_c2, gauss_d3;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  gauss_c1 = 10.0 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3 = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
  gauss_d2_ =
      -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);

  transformation_epsilon_ = 0.1;
  max_iterations_ = 35;
}

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::
    ComputeTransformation(PointCloudSourcePtr output,
                          const Eigen::Matrix4f &guess) {
  apollo::common::time::Timer timer;
  timer.Start();

  nr_iterations_ = 0;
  converged_ = false;
  double gauss_c1, gauss_c2, gauss_d3;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  gauss_c1 = 10 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3 = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
  gauss_d2_ =
      -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity()) {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud(*output, *output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_gradient_.setZero();
  point_gradient_.block<3, 3>(0, 0).setIdentity();
  point_hessian_.setZero();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix() = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation();
  Eigen::Vector3f init_rotation =
      eig_transformation.rotation().eulerAngles(0, 1, 2);
  p << init_translation(0), init_translation(1), init_translation(2),
      init_rotation(0), init_rotation(1), init_rotation(2);

  Eigen::Matrix<double, 6, 6> hessian;
  double score = 0;
  double delta_p_norm;

  // Calculate derivates of initial transform vector, subsequent derivative
  // calculations are done in the step length determination.
  score = ComputeDerivatives(&score_gradient, &hessian, output, &p);
  timer.End("Ndt ComputeDerivatives");

  apollo::common::time::Timer loop_timer;
  loop_timer.Start();
  while (!converged_) {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm
    // 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(
        hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    delta_p = sv.solve(-score_gradient);

    // Calculate step length with guarnteed sufficient decrease [More,
    // Thuente 1994]
    delta_p_norm = delta_p.norm();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {
      trans_probability_ = score / static_cast<double>(input_->points.size());
      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    delta_p.normalize();
    delta_p_norm = ComputeStepLengthMt(p, &delta_p, delta_p_norm, step_size_,
                                       transformation_epsilon_ / 2, &score,
                                       &score_gradient, &hessian, output);
    delta_p *= delta_p_norm;

    transformation_ =
        (Eigen::Translation<float, 3>(static_cast<float>(delta_p(0)),
                                      static_cast<float>(delta_p(1)),
                                      static_cast<float>(delta_p(2))) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(3)),
                                 Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(4)),
                                 Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxis<float>(static_cast<float>(delta_p(5)),
                                 Eigen::Vector3f::UnitZ()))
            .matrix();

    p = p + delta_p;

    if (nr_iterations_ > max_iterations_ ||
        (nr_iterations_ &&
         (std::fabs(delta_p_norm) < transformation_epsilon_))) {
      converged_ = true;
    }

    nr_iterations_++;
  }
  loop_timer.End("Ndt loop.");

  // Store transformation probability.  The relative differences within each
  // scan registration are accurate but the normalization constants need to be
  // modified for it to be globally accurate
  trans_probability_ = score / static_cast<double>(input_->points.size());
}

template <typename PointSource, typename PointTarget>
double
NormalDistributionsTransform<PointSource, PointTarget>::ComputeDerivatives(
    Eigen::Matrix<double, 6, 1> *score_gradient,
    Eigen::Matrix<double, 6, 6> *hessian, PointCloudSourcePtr trans_cloud,
    Eigen::Matrix<double, 6, 1> *p, bool compute_hessian) {
  // Original Point and Transformed Point
  PointSource x_pt, x_trans_pt;
  // Original Point and Transformed Point (for math)
  Eigen::Vector3d x, x_trans;
  // Occupied Voxel
  TargetGridLeafConstPtr cell;
  // Inverse Covariance of Occupied Voxel
  Eigen::Matrix3d c_inv;

  score_gradient->setZero();
  hessian->setZero();
  double score = 0;

  // Precompute Angular Derivatives (eq. 6.19 and 6.21)[Magnusson 2009]
  ComputeAngleDerivatives(*p);

  // Update gradient and hessian for each point, line 17 in Algorithm 2
  // [Magnusson 2009]
  for (size_t idx = 0; idx < input_->points.size(); idx++) {
    x_trans_pt = trans_cloud->points[idx];

    // Find neighbors (Radius search has been experimentally faster than
    // direct neighbor checking.
    std::vector<TargetGridLeafConstPtr> neighborhood;
    std::vector<float> distances;
    target_cells_.RadiusSearch(x_trans_pt, resolution_, &neighborhood,
                               &distances);

    for (typename std::vector<TargetGridLeafConstPtr>::iterator
             neighborhood_it = neighborhood.begin();
         neighborhood_it != neighborhood.end(); neighborhood_it++) {
      cell = *neighborhood_it;
      x_pt = input_->points[idx];
      x = Eigen::Vector3d(x_pt.x, x_pt.y, x_pt.z);

      x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

      // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
      x_trans -= cell->GetMean();
      // Uses precomputed covariance for speed.
      c_inv = cell->GetInverseCov();

      // Compute derivative of transform function w.r.t. transform vector,
      // J_E and H_E in Equations 6.18 and 6.20 [Magnusson 2009]
      ComputePointDerivatives(x);
      // Update score, gradient and hessian, lines 19-21 in Algorithm 2,
      // according to Equations 6.10, 6.12 and 6.13, respectively [Magnusson
      // 2009]
      score += UpdateDerivatives(score_gradient, hessian, x_trans, c_inv,
                                 compute_hessian);
    }
  }
  return (score);
}

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::
    ComputeAngleDerivatives(const Eigen::Matrix<double, 6, 1> &p,
                            bool compute_hessian) {
  // Simplified math for near 0 angles
  double cx, cy, cz, sx, sy, sz;
  if (fabs(p(3)) < 10e-5) {
    // p(3) = 0;
    cx = 1.0;
    sx = 0.0;
  } else {
    cx = cos(p(3));
    sx = sin(p(3));
  }
  if (fabs(p(4)) < 10e-5) {
    // p(4) = 0;
    cy = 1.0;
    sy = 0.0;
  } else {
    cy = cos(p(4));
    sy = sin(p(4));
  }

  if (fabs(p(5)) < 10e-5) {
    // p(5) = 0;
    cz = 1.0;
    sz = 0.0;
  } else {
    cz = cos(p(5));
    sz = sin(p(5));
  }

  // Precomputed angular gradiant components. Letters correspond to
  // Equation 6.19 [Magnusson 2009]
  j_ang_a_ << (-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy);
  j_ang_b_ << (cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy);
  j_ang_c_ << (-sy * cz), sy * sz, cy;
  j_ang_d_ << sx * cy * cz, (-sx * cy * sz), sx * sy;
  j_ang_e_ << (-cx * cy * cz), cx * cy * sz, (-cx * sy);
  j_ang_f_ << (-cy * sz), (-cy * cz), 0;
  j_ang_g_ << (cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0;
  j_ang_h_ << (sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0;

  if (compute_hessian) {
    // Precomputed angular hessian components. Letters correspond to
    // Equation 6.21 and numbers correspond to row index [Magnusson 2009]
    h_ang_a2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy;
    h_ang_a3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz),
        (-cx * cy);

    h_ang_b2_ << (cx * cy * cz), (-cx * cy * sz), (cx * sy);
    h_ang_b3_ << (sx * cy * cz), (-sx * cy * sz), (sx * sy);

    h_ang_c2_ << (-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0;
    h_ang_c3_ << (cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0;

    h_ang_d1_ << (-cy * cz), (cy * sz), (sy);
    h_ang_d2_ << (-sx * sy * cz), (sx * sy * sz), (sx * cy);
    h_ang_d3_ << (cx * sy * cz), (-cx * sy * sz), (-cx * cy);

    h_ang_e1_ << (sy * sz), (sy * cz), 0;
    h_ang_e2_ << (-sx * cy * sz), (-sx * cy * cz), 0;
    h_ang_e3_ << (cx * cy * sz), (cx * cy * cz), 0;

    h_ang_f1_ << (-cy * cz), (cy * sz), 0;
    h_ang_f2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0;
    h_ang_f3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0;
  }
}

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<
    PointSource, PointTarget>::ComputePointDerivatives(const Eigen::Vector3d &x,
                                                       bool compute_hessian) {
  // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform
  // vector p. Derivative w.r.t. ith element of transform vector corresponds to
  // column i, Equation 6.18 and 6.19 [Magnusson 2009]
  point_gradient_(1, 3) = x.dot(j_ang_a_);
  point_gradient_(2, 3) = x.dot(j_ang_b_);
  point_gradient_(0, 4) = x.dot(j_ang_c_);
  point_gradient_(1, 4) = x.dot(j_ang_d_);
  point_gradient_(2, 4) = x.dot(j_ang_e_);
  point_gradient_(0, 5) = x.dot(j_ang_f_);
  point_gradient_(1, 5) = x.dot(j_ang_g_);
  point_gradient_(2, 5) = x.dot(j_ang_h_);

  if (compute_hessian) {
    // Vectors from Equation 6.21 [Magnusson 2009]
    Eigen::Vector3d a, b, c, d, e, f;

    a << 0, x.dot(h_ang_a2_), x.dot(h_ang_a3_);
    b << 0, x.dot(h_ang_b2_), x.dot(h_ang_b3_);
    c << 0, x.dot(h_ang_c2_), x.dot(h_ang_c3_);
    d << x.dot(h_ang_d1_), x.dot(h_ang_d2_), x.dot(h_ang_d3_);
    e << x.dot(h_ang_e1_), x.dot(h_ang_e2_), x.dot(h_ang_e3_);
    f << x.dot(h_ang_f1_), x.dot(h_ang_f2_), x.dot(h_ang_f3_);

    // Calculate second derivative of Transformation Equation 6.17 w.r.t.
    // transform vector p. Derivative w.r.t. ith and jth elements of transform
    // vector corresponds to the 3x1 block matrix starting at (3i,j),
    // Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian_.block<3, 1>(9, 3) = a;
    point_hessian_.block<3, 1>(12, 3) = b;
    point_hessian_.block<3, 1>(15, 3) = c;
    point_hessian_.block<3, 1>(9, 4) = b;
    point_hessian_.block<3, 1>(12, 4) = d;
    point_hessian_.block<3, 1>(15, 4) = e;
    point_hessian_.block<3, 1>(9, 5) = c;
    point_hessian_.block<3, 1>(12, 5) = e;
    point_hessian_.block<3, 1>(15, 5) = f;
  }
}

template <typename PointSource, typename PointTarget>
double
NormalDistributionsTransform<PointSource, PointTarget>::UpdateDerivatives(
    Eigen::Matrix<double, 6, 1> *score_gradient,
    Eigen::Matrix<double, 6, 6> *hessian, const Eigen::Vector3d &x_trans,
    const Eigen::Matrix3d &c_inv, bool compute_hessian) {
  Eigen::Vector3d cov_dxd_pi;
  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson
  // 2009]
  double e_x_cov_x = exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
  // Calculate probability of transtormed points existence, Equation 6.9
  // [Magnusson 2009]
  double score_inc = -gauss_d1_ * e_x_cov_x;

  e_x_cov_x = gauss_d2_ * e_x_cov_x;

  // Error checking for invalid values.
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x) {
    return 0;
  }

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  e_x_cov_x *= gauss_d1_;

  for (int i = 0; i < 6; i++) {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13
    // [Magnusson 2009]
    cov_dxd_pi = c_inv * point_gradient_.col(i);

    // Update gradient, Equation 6.12 [Magnusson 2009]
    (*score_gradient)(i) += x_trans.dot(cov_dxd_pi) * e_x_cov_x;

    if (compute_hessian) {
      for (int j = 0; j < hessian->cols(); j++) {
        // Update hessian, Equation 6.13 [Magnusson 2009]
        (*hessian)(i, j) +=
            e_x_cov_x *
            (-gauss_d2_ * x_trans.dot(cov_dxd_pi) *
                 x_trans.dot(c_inv * point_gradient_.col(j)) +
             x_trans.dot(c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
             point_gradient_.col(j).dot(cov_dxd_pi));
      }
    }
  }

  return score_inc;
}

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::ComputeHessian(
    Eigen::Matrix<double, 6, 6> *hessian, const PointCloudSource &trans_cloud,
    Eigen::Matrix<double, 6, 1> *p) {
  // Original Point and Transformed Point
  PointSource x_pt, x_trans_pt;
  // Original Point and Transformed Point (for math)
  Eigen::Vector3d x, x_trans;
  // Occupied Voxel
  TargetGridLeafConstPtr cell;
  // Inverse Covariance of Occupied Voxel
  Eigen::Matrix3d c_inv;

  hessian->setZero();

  // Precompute Angular Derivatives unnecessary because only used after regular
  // derivative calculation

  // Update hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (size_t idx = 0; idx < input_->points.size(); idx++) {
    x_trans_pt = trans_cloud.points[idx];

    // Find neighbors (Radius search has been experimentally faster than
    // direct neighbor checking.
    std::vector<TargetGridLeafConstPtr> neighborhood;
    std::vector<float> distances;
    target_cells_.RadiusSearch(x_trans_pt, resolution_, &neighborhood,
                               &distances);

    for (typename std::vector<TargetGridLeafConstPtr>::iterator
             neighborhood_it = neighborhood.begin();
         neighborhood_it != neighborhood.end(); neighborhood_it++) {
      cell = *neighborhood_it;
      {
        x_pt = input_->points[idx];
        x = Eigen::Vector3d(x_pt.x, x_pt.y, x_pt.z);

        x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

        // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson
        // 2009]
        x_trans -= cell->GetMean();
        // Uses precomputed covariance for speed.
        c_inv = cell->GetInverseCov();

        // Compute derivative of transform function w.r.t. transform
        // vector, J_E and H_E in Equations 6.18 and 6.20 [Magnusson
        // 2009]
        ComputePointDerivatives(x);
        // Update hessian, lines 21 in Algorithm 2, according to
        // Equations 6.10, 6.12 and 6.13, respectively [Magnusson 2009]
        UpdateHessian(hessian, x_trans, c_inv);
      }
    }
  }
}

template <typename PointSource, typename PointTarget>
void NormalDistributionsTransform<PointSource, PointTarget>::UpdateHessian(
    Eigen::Matrix<double, 6, 6> *hessian, const Eigen::Vector3d &x_trans,
    const Eigen::Matrix3d &c_inv) {
  Eigen::Vector3d cov_dxd_pi;
  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9
  // [Magnusson 2009]
  double e_x_cov_x =
      gauss_d2_ * exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);

  // Error checking for invalid values.
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x) {
    return;
  }

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  e_x_cov_x *= gauss_d1_;

  for (int i = 0; i < 6; i++) {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13
    // [Magnusson 2009]
    cov_dxd_pi = c_inv * point_gradient_.col(i);

    for (int j = 0; j < hessian->cols(); j++) {
      // Update hessian, Equation 6.13 [Magnusson 2009]
      (*hessian)(i, j) +=
          e_x_cov_x *
          (-gauss_d2_ * x_trans.dot(cov_dxd_pi) *
               x_trans.dot(c_inv * point_gradient_.col(j)) +
           x_trans.dot(c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
           point_gradient_.col(j).dot(cov_dxd_pi));
    }
  }
}

template <typename PointSource, typename PointTarget>
bool NormalDistributionsTransform<PointSource, PointTarget>::UpdateIntervalMt(
    double *a_l, double *f_l, double *g_l, double *a_u, double *f_u,
    double *g_u, double a_t, double f_t, double g_t) {
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm
  // [More, Thuente 1994]
  if (f_t > (*f_l)) {
    (*a_u) = a_t;
    (*f_u) = f_t;
    (*g_u) = g_t;
    return (false);
  } else if (g_t * ((*a_l) - a_t) > 0) {
    // Case U2 in Update Algorithm and Case b in Modified Update Algorithm
    // [More, Thuente 1994]
    (*a_l) = a_t;
    (*f_l) = f_t;
    (*g_l) = g_t;
    return (false);
  } else if (g_t * ((*a_l) - a_t) < 0) {
    // Case U3 in Update Algorithm and Case c in Modified Update Algorithm
    // [More, Thuente 1994]
    (*a_u) = (*a_l);
    (*f_u) = (*f_l);
    (*g_u) = (*g_l);

    (*a_l) = a_t;
    (*f_l) = f_t;
    (*g_l) = g_t;
    return (false);
  }
  return (true);
}

template <typename PointSource, typename PointTarget>
double
NormalDistributionsTransform<PointSource, PointTarget>::TrialValueSelectionMt(
    double a_l, double f_l, double g_l, double a_u, double f_u, double g_u,
    double a_t, double f_t, double g_t) {
  // Case 1 in Trial Value Selection [More, Thuente 1994]
  if (f_t > f_l) {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and
    // g_t Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, f_t and
    // g_l Equation 2.4.2 [Sun, Yuan 2006]
    double a_q =
        a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

    if (std::fabs(a_c - a_l) < std::fabs(a_q - a_l))
      return (a_c);
    else
      return (0.5 * (a_q + a_c));
  } else if (g_t * g_l < 0) {
    // Case 2 in Trial Value Selection [More, Thuente 1994]
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and
    // g_t Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, g_l and
    // g_t Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    if (std::fabs(a_c - a_t) >= std::fabs(a_s - a_t))
      return (a_c);
    else
      return (a_s);
  } else if (std::fabs(g_t) <= std::fabs(g_l)) {
    // Case 3 in Trial Value Selection [More, Thuente 1994]
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and
    // g_t Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates g_l and g_t
    // Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    double a_t_next;

    if (std::fabs(a_c - a_t) < std::fabs(a_s - a_t))
      a_t_next = a_c;
    else
      a_t_next = a_s;

    if (a_t > a_l)
      return (std::min(a_t + 0.66 * (a_u - a_t), a_t_next));
    else
      return (std::max(a_t + 0.66 * (a_u - a_t), a_t_next));
  } else {
    // Case 4 in Trial Value Selection [More, Thuente 1994]
    // Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u  and
    // g_t Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
    double w = std::sqrt(z * z - g_t * g_u);
    // Equation 2.4.56 [Sun, Yuan 2006]
    return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w));
  }
}

template <typename PointSource, typename PointTarget>
double
NormalDistributionsTransform<PointSource, PointTarget>::ComputeStepLengthMt(
    const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> *step_dir,
    double step_init, double step_max, double step_min, double *score,
    Eigen::Matrix<double, 6, 1> *score_gradient,
    Eigen::Matrix<double, 6, 6> *hessian, PointCloudSourcePtr trans_cloud) {
  // Set the value of phi(0), Equation 1.3 [More, Thuente 1994]
  double phi_0 = -(*score);
  // Set the value of phi'(0), Equation 1.3 [More, Thuente 1994]
  double d_phi_0 = -(score_gradient->dot(*step_dir));

  Eigen::Matrix<double, 6, 1> x_t;

  if (d_phi_0 >= 0) {
    // Not a decent direction
    if (d_phi_0 == 0) {
      return 0;
    } else {
      // Reverse step direction and calculate optimal step.
      d_phi_0 *= -1;
      (*step_dir) *= -1;
    }
  }

  // The Search Algorithm for T(mu) [More, Thuente 1994]

  int max_step_iterations = 10;
  int step_iterations = 0;

  // Sufficient decreace constant, Equation 1.1 [More, Thuete 1994]
  double mu = 1.e-4;
  // Curvature condition constant, Equation 1.2 [More, Thuete 1994]
  double nu = 0.9;

  // Initial endpoints of Interval I,
  double a_l = 0, a_u = 0;

  // Auxiliary function psi is used until I is determined ot be a closed
  // interval, Equation 2.1 [More, Thuente 1994]
  double f_l = AuxilaryFunctionPsimt(a_l, phi_0, phi_0, d_phi_0, mu);
  double g_l = AuxilaryFunctionDpsimt(d_phi_0, d_phi_0, mu);

  double f_u = AuxilaryFunctionPsimt(a_u, phi_0, phi_0, d_phi_0, mu);
  double g_u = AuxilaryFunctionDpsimt(d_phi_0, d_phi_0, mu);

  // Check used to allow More-Thuente step length calculation to be skipped by
  // making step_min == step_max
  bool interval_converged = (step_max - step_min) > 0, open_interval = true;

  double a_t = step_init;
  a_t = std::min(a_t, step_max);
  a_t = std::max(a_t, step_min);

  x_t = x + (*step_dir) * a_t;

  final_transformation_ =
      (Eigen::Translation<float, 3>(static_cast<float>(x_t(0)),
                                    static_cast<float>(x_t(1)),
                                    static_cast<float>(x_t(2))) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(3)),
                               Eigen::Vector3f::UnitX()) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(4)),
                               Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(5)),
                               Eigen::Vector3f::UnitZ()))
          .matrix();

  // New transformed point cloud
  transformPointCloud(*input_, *trans_cloud, final_transformation_);

  // Updates score, gradient and hessian.  Hessian calculation is unnecessary
  // but testing showed that most step calculations use the initial step
  // suggestion and recalculation the reusable portions of the hessian would
  // intail more computation time.
  (*score) =
      ComputeDerivatives(score_gradient, hessian, trans_cloud, &x_t, true);

  // Calculate phi(alpha_t)
  double phi_t = -(*score);
  // Calculate phi'(alpha_t)
  double d_phi_t = -(score_gradient->dot(*step_dir));

  // Calculate psi(alpha_t)
  double psi_t = AuxilaryFunctionPsimt(a_t, phi_t, phi_0, d_phi_0, mu);
  // Calculate psi'(alpha_t)
  double d_psi_t = AuxilaryFunctionDpsimt(d_phi_t, d_phi_0, mu);

  // Iterate until max number of iterations, interval convergance or a value
  // satisfies the sufficient decrease, Equation 1.1, and curvature condition,
  // Equation 1.2 [More, Thuente 1994]
  while (!interval_converged && step_iterations < max_step_iterations &&
         !(psi_t <= 0 /*Sufficient Decrease*/ &&
           d_phi_t <= -nu * d_phi_0 /*Curvature Condition*/)) {
    // Use auxiliary function if interval I is not closed
    if (open_interval) {
      a_t = TrialValueSelectionMt(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t,
                                  d_psi_t);
    } else {
      a_t = TrialValueSelectionMt(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t,
                                  d_phi_t);
    }

    a_t = std::min(a_t, step_max);
    a_t = std::max(a_t, step_min);

    x_t = x + (*step_dir) * a_t;

    final_transformation_ =
        (Eigen::Translation<float, 3>(static_cast<float>(x_t(0)),
                                      static_cast<float>(x_t(1)),
                                      static_cast<float>(x_t(2))) *
         Eigen::AngleAxis<float>(static_cast<float>(x_t(3)),
                                 Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxis<float>(static_cast<float>(x_t(4)),
                                 Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxis<float>(static_cast<float>(x_t(5)),
                                 Eigen::Vector3f::UnitZ()))
            .matrix();

    // New transformed point cloud
    // Done on final cloud to prevent wasted computation
    transformPointCloud(*input_, *trans_cloud, final_transformation_);

    // Updates score, gradient. Values stored to prevent wasted computation.
    *score =
        ComputeDerivatives(score_gradient, hessian, trans_cloud, &x_t, false);

    // Calculate phi(alpha_t+)
    phi_t = -(*score);
    // Calculate phi'(alpha_t+)
    d_phi_t = -(score_gradient->dot(*step_dir));

    // Calculate psi(alpha_t+)
    psi_t = AuxilaryFunctionPsimt(a_t, phi_t, phi_0, d_phi_0, mu);
    // Calculate psi'(alpha_t+)
    d_psi_t = AuxilaryFunctionDpsimt(d_phi_t, d_phi_0, mu);

    // Check if I is now a closed interval
    if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
      open_interval = false;

      // Converts f_l and g_l from psi to phi
      f_l = f_l + phi_0 - mu * d_phi_0 * a_l;
      g_l = g_l + mu * d_phi_0;

      // Converts f_u and g_u from psi to phi
      f_u = f_u + phi_0 - mu * d_phi_0 * a_u;
      g_u = g_u + mu * d_phi_0;
    }

    if (open_interval) {
      // Update interval end points using Updating Algorithm [More,
      // Thuente 1994]
      interval_converged = UpdateIntervalMt(&a_l, &f_l, &g_l, &a_u, &f_u, &g_u,
                                            a_t, psi_t, d_psi_t);
    } else {
      // Update interval end points using Modified Updating Algorithm
      // [More, Thuente 1994]
      interval_converged = UpdateIntervalMt(&a_l, &f_l, &g_l, &a_u, &f_u, &g_u,
                                            a_t, phi_t, d_phi_t);
    }

    step_iterations++;
  }

  // If inner loop was run then hessian needs to be calculated.
  // Hessian is unnecessary for step length determination but gradients are
  // required so derivative and transform data is stored for the next
  // iteration.
  if (step_iterations) ComputeHessian(hessian, *trans_cloud, &x_t);

  return a_t;
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
