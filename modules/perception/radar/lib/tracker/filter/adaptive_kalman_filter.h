// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: hui yujiang (huiyujiang@baidu.com)
// @file: adaptive_kalman_filter.h
// @brief: adaptive kalman filter.
#ifndef RADAR_LIB_TRACKER_FILTER_ADAPTIVE_KALMAN_FILTER_H_
#define RADAR_LIB_TRACKER_FILTER_ADAPTIVE_KALMAN_FILTER_H_

#include <vector>
#include "modules/perception/radar/lib/interface/base_filter.h"

namespace apollo {
namespace perception {
namespace radar {
class AdaptiveKalmanFilter : public BaseFilter {
 public:
  AdaptiveKalmanFilter();
  ~AdaptiveKalmanFilter();
  void Init(const base::Object& object) override;
  Eigen::VectorXd Predict(const double time_diff) override;
  Eigen::VectorXd UpdateWithObject(const base::Object& new_object,
                                   double time_diff) override;
  void GetState(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity);
  Eigen::Matrix4d GetCovarianceMatrix() override { return p_matrix_; }
  static void SetQMatrixRatio(double q_matrix_ratio) {
    s_q_matrix_ratio_ = q_matrix_ratio;
  }

 private:
  Eigen::Vector3d belief_anchor_point_;
  Eigen::Vector3d belief_velocity_;
  Eigen::Vector4d priori_state_;
  Eigen::Vector4d posteriori_state_;
  Eigen::Matrix4d p_matrix_;
  // the state-transition matrix
  Eigen::Matrix4d a_matrix_;
  // the observation mode
  Eigen::Matrix4d c_matrix_;
  // the covariance of the process noise
  Eigen::Matrix4d q_matrix_;
  //  the covariance of the observation noise
  Eigen::Matrix4d r_matrix_;
  // Optimal Kalman gain
  Eigen::Matrix4d k_matrix_;
  static double s_q_matrix_ratio_;
};
}  // namespace radar
}  // namespace perception
}  // namespace apollo
#endif  // RADAR_LIB_TRACKER_FILTER_ADAPTIVE_KALMAN_FILTER_H_
