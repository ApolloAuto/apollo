/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar_detection/lib/tracker/filter/adaptive_kalman_filter.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(AdaptiveKalmanFilterTest, adaptive_kalman_filter_test) {
  BaseFilter* filter = new AdaptiveKalmanFilter();
  EXPECT_NE(filter, nullptr);
  EXPECT_EQ(filter->Name(), "AdaptiveKalmanFilter");
  const double time_diff = 0.074;
  base::Object object;
  object.velocity << 3.0f, 4.0f, 0.0f;
  object.center << 0.0f, 0.0f, 0.0f;
  object.anchor_point = object.center;
  Eigen::Matrix3d temp_matrix;
  temp_matrix.setIdentity();
  temp_matrix *= 0.01;
  object.center_uncertainty = temp_matrix.cast<float>();
  object.velocity_uncertainty = temp_matrix.cast<float>();
  AdaptiveKalmanFilter::SetQMatrixRatio(0.074);
  filter->Init(object);
  // predict
  Eigen::Vector4d predict_state = filter->Predict(time_diff);
  AINFO << std::setprecision(12) << predict_state;
  Eigen::Vector4d result;
  result[0] = object.anchor_point(0) + object.velocity(0) * time_diff;
  result[1] = object.anchor_point(1) + object.velocity(1) * time_diff;
  result[2] = object.velocity(0);
  result[3] = object.velocity(1);
  EXPECT_LT(std::fabs(predict_state(0) - result(0)), 1e-5);
  EXPECT_LT(std::fabs(predict_state(1) - result(1)), 1e-5);
  EXPECT_LT(std::fabs(predict_state(2) - result(2)), 1e-5);
  EXPECT_LT(std::fabs(predict_state(3) - result(3)), 1e-5);
  // update (predict & correct)
  base::Object object2;
  object2.velocity << 3.01f, 3.99f, 0.0f;
  object2.center << 0.0f, 0.0f, 0.0f;
  object2.anchor_point = object.center;
  object2.center_uncertainty = temp_matrix.cast<float>();
  object2.velocity_uncertainty = temp_matrix.cast<float>();
  Eigen::Vector4d update_result = filter->UpdateWithObject(object2, time_diff);
  Eigen::Vector3d anchor_point;
  Eigen::Vector3d velocity;
  filter->GetState(&anchor_point, &velocity);
  EXPECT_LT((update_result.head(2) - anchor_point.head(2)).norm(), 1e-5);
  EXPECT_LT((update_result.tail(2) - velocity.head(2)).norm(), 1e-5);
  AINFO << "upate_result:" << std::setprecision(12) << update_result;
  result[0] = 0.02361;
  result[1] = 0.03146;
  result[2] = 3.00875;
  result[3] = 3.99082;
  EXPECT_LT(std::fabs(update_result(0) - result(0)), 1e-3);
  EXPECT_LT(std::fabs(update_result(1) - result(1)), 1e-3);
  EXPECT_LT(std::fabs(update_result(2) - result(2)), 1e-3);
  EXPECT_LT(std::fabs(update_result(3) - result(3)), 1e-3);
  delete filter;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
