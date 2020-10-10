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
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <limits>
#include <vector>

#include "pcl/registration/registration.h"
#include "unsupported/Eigen/NonLinearOptimization"

#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/localization/ndt/ndt_locator/ndt_voxel_grid_covariance.h"

namespace apollo {
namespace localization {
namespace ndt {

template <typename PointSource, typename PointTarget>
class NormalDistributionsTransform {
 protected:
  /**@brief Typename of source point. */
  typedef typename pcl::PointCloud<PointSource> PointCloudSource;
  typedef typename boost::shared_ptr<PointCloudSource> PointCloudSourcePtr;
  typedef boost::shared_ptr<const PointCloudSource> PointCloudSourceConstPtr;

  /**@brief Typename of target point. */
  typedef typename pcl::PointCloud<PointTarget> PointCloudTarget;
  typedef boost::shared_ptr<const PointCloudTarget> PointCloudTargetConstPtr;

  /**@brief Typename of searchable voxel grid containing mean and covariance. */
  typedef VoxelGridCovariance<PointTarget> TargetGrid;
  typedef TargetGrid *TargetGridPtr;
  typedef const TargetGrid *TargetGridConstPtr;
  typedef LeafConstPtr TargetGridLeafConstPtr;

  /**@brief Typename of KD-Tree. */
  typedef pcl::search::KdTree<PointTarget> KdTree;
  typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

 public:
  /**@brief Typedef shared pointer. */
  typedef boost::shared_ptr<
      NormalDistributionsTransform<PointSource, PointTarget>>
      Ptr;
  typedef boost::shared_ptr<
      const NormalDistributionsTransform<PointSource, PointTarget>>
      ConstPtr;

  /**@brief Constructor. */
  NormalDistributionsTransform();

  /**@brief Destructor. */
  ~NormalDistributionsTransform() {
    target_.reset();
    input_.reset();
  }

  /**@brief Provide a pointer to the input target. */
  inline void SetInputTarget(const std::vector<Leaf> &cell_leaf,
                             const PointCloudTargetConstPtr &cloud) {
    if (cell_leaf.empty()) {
      AWARN << "Input leaf is empty.";
      return;
    }
    if (cloud->points.empty()) {
      AWARN << "Input target is empty.";
      return;
    }

    target_ = cloud;
    target_cells_.SetVoxelGridResolution(resolution_, resolution_, resolution_);
    target_cells_.SetInputCloud(cloud);
    target_cells_.filter(cell_leaf, true);
  }

  /**@brief Provide a pointer to the input target. */
  inline void SetInputSource(const PointCloudTargetConstPtr &cloud) {
    if (cloud->points.empty()) {
      AWARN << "Input source is empty.";
      return;
    }
    input_ = cloud;
  }

  /**@brief Set/change the voxel grid resolution. */
  inline void SetResolution(float resolution) {
    // Prevents unnessary voxel initiations
    if (resolution_ != resolution) {
      resolution_ = resolution;
    }
    AINFO << "NDT Resolution: " << resolution_;
  }

  /**@brief Get voxel grid resolution. */
  inline float GetResolution() const { return resolution_; }

  /**@brief Get the newton line search maximum step length.
   * \return maximum step length
   */
  inline double GetStepSize() const { return step_size_; }

  /**@brief Set/change the newton line search maximum step length.
   * \param[in] step_size maximum step length
   */
  inline void SetStepSize(double step_size) { step_size_ = step_size; }

  /**@brief Get the point cloud outlier ratio. */
  inline double GetOulierRatio() const { return outlier_ratio_; }

  /**@brief Set/change the point cloud outlier ratio. */
  inline void SetOulierRatio(double outlier_ratio) {
    outlier_ratio_ = outlier_ratio;
  }

  /**@brief Get the registration alignment probability. */
  inline double GetTransformationProbability() const {
    return trans_probability_;
  }

  /**@brief Get the number of iterations required to calculate alignment. */
  inline int GetFinalNumIteration() const { return nr_iterations_; }

  /**@brief Return the state of convergence after the last align run. */
  inline bool HasConverged() const { return converged_; }

  /**@brief Get the final transformation matrix estimated by the
   * registration method. */
  inline Eigen::Matrix4f GetFinalTransformation() const {
    return final_transformation_;
  }

  /**@brief Set the maximum number of iterations the internal optimization
   * should run for.*/
  inline void SetMaximumIterations(int nr_iterations) {
    max_iterations_ = nr_iterations;
  }

  /**@brief Set the transformation epsilon (maximum allowable difference
   * between two consecutive transformations. */
  inline void SetTransformationEpsilon(double epsilon) {
    transformation_epsilon_ = epsilon;
  }

  /**@brief Convert 6 element transformation vector to affine transformation. */
  static void ConvertTransform(const Eigen::Matrix<double, 6, 1> &x,
                               Eigen::Affine3f *trans) {
    *trans = Eigen::Translation<float, 3>(static_cast<float>(x(0)),
                                          static_cast<float>(x(1)),
                                          static_cast<float>(x(2))) *
             Eigen::AngleAxis<float>(static_cast<float>(x(3)),
                                     Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxis<float>(static_cast<float>(x(4)),
                                     Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxis<float>(static_cast<float>(x(5)),
                                     Eigen::Vector3f::UnitZ());
  }

  /**@brief Convert 6 element transformation vector to transformation matrix. */
  static void ConvertTransform(const Eigen::Matrix<double, 6, 1> &x,
                               Eigen::Matrix4f *trans) {
    Eigen::Affine3f _affine;
    ConvertTransform(x, &_affine);
    *trans = _affine.matrix();
  }

  /**@brief Obtain probability point cloud. */
  void GetGridPointCloud(pcl::PointCloud<pcl::PointXYZ> *cell_cloud) {
    target_cells_.GetDisplayCloud(*cell_cloud);
  }

  /**@brief Set left top corner of target point cloud. */
  inline void SetLeftTopCorner(const Eigen::Vector3d &left_top_corner) {
    target_cells_.SetMapLeftTopCorner(left_top_corner);
  }

  /**@brief Obtain the Euclidean fitness score. */
  double GetFitnessScore(double max_range = std::numeric_limits<double>::max());

  /**@brief Call the registration algorithm which estimates the
   * transformation and returns the transformed source. */
  void Align(PointCloudSourcePtr output, const Eigen::Matrix4f &guess);

 protected:
  /**@brief Estimate the transformation and returns the transformed source
   * (input) as output. */
  void ComputeTransformation(PointCloudSourcePtr output) {
    ComputeTransformation(output, Eigen::Matrix4f::Identity());
  }

  /**@brief Estimate the transformation and returns the transformed source
   * (input) as output. */
  void ComputeTransformation(PointCloudSourcePtr output,
                             const Eigen::Matrix4f &guess);

  /**@brief Compute derivatives of probability function w.r.t. the
   * transformation vector. */
  double ComputeDerivatives(Eigen::Matrix<double, 6, 1> *score_gradient,
                            Eigen::Matrix<double, 6, 6> *hessian,
                            PointCloudSourcePtr trans_cloud,
                            Eigen::Matrix<double, 6, 1> *p,
                            bool ComputeHessian = true);

  /**@brief Compute individual point contributions to derivatives of
   * probability function w.r.t. the transformation vector. */
  double UpdateDerivatives(Eigen::Matrix<double, 6, 1> *score_gradient,
                           Eigen::Matrix<double, 6, 6> *hessian,
                           const Eigen::Vector3d &x_trans,
                           const Eigen::Matrix3d &c_inv,
                           bool ComputeHessian = true);

  /**@brief Precompute anglular components of derivatives. */
  void ComputeAngleDerivatives(const Eigen::Matrix<double, 6, 1> &p,
                               bool ComputeHessian = true);

  /**@brief Compute point derivatives. */
  void ComputePointDerivatives(const Eigen::Vector3d &x,
                               bool ComputeHessian = true);

  /**@brief Compute hessian of probability function w.r.t. the transformation
   * vector. */
  void ComputeHessian(Eigen::Matrix<double, 6, 6> *hessian,
                      const PointCloudSource &trans_cloud,
                      Eigen::Matrix<double, 6, 1> *p);

  /**@brief Compute individual point contributions to hessian of probability
   * function. */
  void UpdateHessian(Eigen::Matrix<double, 6, 6> *hessian,
                     const Eigen::Vector3d &x_trans,
                     const Eigen::Matrix3d &c_inv);

  /**@brief Compute line search step length and update transform and probability
   * derivatives. */
  double ComputeStepLengthMt(const Eigen::Matrix<double, 6, 1> &x,
                             Eigen::Matrix<double, 6, 1> *step_dir,
                             double step_init, double step_max, double step_min,
                             double *score,
                             Eigen::Matrix<double, 6, 1> *score_gradient,
                             Eigen::Matrix<double, 6, 6> *hessian,
                             PointCloudSourcePtr trans_cloud);

  /**@brief Update interval of possible step lengths for More-Thuente method. */
  bool UpdateIntervalMt(double *a_l, double *f_l, double *g_l, double *a_u,
                        double *f_u, double *g_u, double a_t, double f_t,
                        double g_t);

  /**@brief Select new trial value for More-Thuente method. */
  double TrialValueSelectionMt(double a_l, double f_l, double g_l, double a_u,
                               double f_u, double g_u, double a_t, double f_t,
                               double g_t);

  /**@brief Auxiliary function used to determine endpoints of More-Thuente
   * interval. */
  inline double AuxilaryFunctionPsimt(double a, double f_a, double f_0,
                                      double g_0, double mu = 1.e-4) {
    return (f_a - f_0 - mu * g_0 * a);
  }

  /**@brief Auxiliary function derivative used to determine endpoints of
   * More-Thuente interval. */
  inline double AuxilaryFunctionDpsimt(double g_a, double g_0,
                                       double mu = 1.e-4) {
    return (g_a - mu * g_0);
  }

 protected:
  /**@brief A pointer of input target point cloud. */
  PointCloudTargetConstPtr target_;
  /**@brief A pointer to the spatial search object. */
  KdTreePtr target_tree_;
  /**@brief The voxel grid generated from target cloud containing point
   * means and covariances. */
  TargetGrid target_cells_;

  /**@brief A pointer of input source point cloud. */
  PointCloudSourceConstPtr input_;

  /**@brief finnal iterations. */
  int nr_iterations_;
  bool converged_;
  /**@brief transformations. */
  Eigen::Matrix4f previous_transformation_;
  Eigen::Matrix4f final_transformation_;
  Eigen::Matrix4f transformation_;

  /**@brief max iterations. */
  int max_iterations_;
  /**@brief Transformation epsilon parameter. */
  double transformation_epsilon_;
  /**@brief The side length of voxels. */
  float resolution_;
  /**@brief The maximum step length. */
  double step_size_;
  /**@brief The ratio of outliers of points w.r.t. a normal distribution,
   * Equation 6.7 [Magnusson 2009]. */
  double outlier_ratio_;
  /**@brief The normalization constants used fit the point distribution to a
   * normal distribution, Equation 6.8 [Magnusson 2009]. */
  double gauss_d1_, gauss_d2_;
  /**@brief The probability score of the transform applied to the input
   * cloud, Equation 6.9 and 6.10 [Magnusson 2009]. */
  double trans_probability_;

  /**@brief The precomputed angular derivatives for the jacobian of a
   * transformation vector, Equation 6.19 [Magnusson 2009]. */
  Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_,
      j_ang_g_, j_ang_h_;
  /**@brief The precomputed angular derivatives for the hessian of a
   * transformation vector, Equation 6.19 [Magnusson 2009]. */
  Eigen::Vector3d h_ang_a2_, h_ang_a3_, h_ang_b2_, h_ang_b3_, h_ang_c2_,
      h_ang_c3_, h_ang_d1_, h_ang_d2_, h_ang_d3_, h_ang_e1_, h_ang_e2_,
      h_ang_e3_, h_ang_f1_, h_ang_f2_, h_ang_f3_;
  /**@brief The first order derivative of the transformation of a point
   * w.r.t. the transform vector, Equation 6.18 [Magnusson 2009]. */
  Eigen::Matrix<double, 3, 6> point_gradient_;
  /**@brief The second order derivative of the transformation of a point
   * w.r.t. the transform vector, Equation 6.20 [Magnusson 2009]. */
  Eigen::Matrix<double, 18, 6> point_hessian_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ndt
}  // namespace localization
}  // namespace apollo

#include "modules/localization/ndt/ndt_locator/ndt_solver.hpp"
