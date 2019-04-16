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

#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/half_circle_angle.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/kalman_filter.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/similar.h"
#include "modules/perception/inference/utils/cuda_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(KalmanTest, copy_test) {
  KalmanFilterConstVelocity kf;
  Eigen::Vector3d center;
  Eigen::Vector2d measure;
  center << 0, 0, 0;
  kf.Init(center);
  for (int i = 0; i < 10; ++i) {
    kf.Predict(10);
    measure << i, i * 2;
    kf.Correct(measure);
  }
  KalmanFilterConstVelocity kf2 = kf;
  EXPECT_EQ(kf2.get_state(), kf.get_state());
}

TEST(KalmanTest, zero_test) {
  KalmanFilterConstVelocity kf;
  Eigen::Vector3d center;
  Eigen::Vector4d state;
  Eigen::Vector2d measure;
  center << 0, 0, 0;
  state << 0, 0, 0, 0;
  measure << 0, 0;
  kf.Init(center);
  for (int i = 0; i < 10; ++i) {
    kf.Predict(10);
    kf.Correct(measure);
    EXPECT_EQ(state, kf.get_state());
  }
}

TEST(KalmanTest, kalman_test) {
  KalmanFilterConstVelocity kf;
  // x is ground truth
  // z is observation
  // state is filter result
  Eigen::Vector2d x;
  Eigen::Vector2d speed;
  Eigen::Vector2d z;
  x << 0, 0;
  speed << 0.1, 0.5;
  kf.Init(x);
  kf.measure_noise_ *= 25;
  for (int i = 0; i < 100; ++i) {
    x = x + speed;
    speed += Eigen::Vector2d::Random() * 1e-4;
    z = x + Eigen::Vector2d::Random() * 1e-4 * 5;
    kf.Predict(1);
    kf.Correct(z);
    Eigen::Vector4d state = kf.get_state();
    if (i > 10) {
      EXPECT_LT(std::fabs(state[2] - speed[0]), 0.05);
      EXPECT_LT(std::fabs(state[3] - speed[1]), 0.05);
      EXPECT_LT(std::fabs(state[0] - x[0]), 0.05);
      EXPECT_LT(std::fabs(state[1] - x[1]), 0.05);
    }
  }
}

TEST(EKFTest, ekf_test) {
  //  ExtendedKalmanFilter ekf;
  //  Eigen::Vector3d x;
  //  Eigen::Vector3d z;  // observation: x, y, theta
  //  float speed = 3.0f;
  //  float theta = 0.5f;
  //  ekf.Init();
  //  ekf.measure_noise_ *= 5;
  //  ekf.Predict(1.f);
  //  x << 0.f, 0.f, theta;
  //  z << 0.f, 0.f, theta;
  //  ekf.Correct(z);
  //  Eigen::Vector4d state = ekf.get_state();
  //  ASSERT_TRUE(std::fabs(state[0]) < 1e-6);
  //  ASSERT_TRUE(std::fabs(state[1]) < 1e-6);
  //  ASSERT_TRUE(std::fabs(state[2]) < 1e-6);
  //  ASSERT_TRUE(std::fabs(state[3] - 0.5f) < 1e-6);
  //
  //  auto gen_rand_number = [](float l, float h) {
  //    return l + static_cast<float>(rand()) /    // NOLINT
  //        (static_cast<float>(RAND_MAX / (h - l)));
  //  };
  //
  //  for (int i = 0; i < 100; ++i) {
  //    // speed = 3 m/s, delta_x = speed * cos(theta)
  //    // delta_y = speed * sin(theta)
  //    x[0] += speed * std::cos(theta);
  //    x[1] += speed * std::sin(theta);
  //    z << x[0], x[1], x[2];
  //    speed += 0.01;
  //    theta += gen_rand_number(-0.01, 0.01);
  //
  //    ekf.Predict(1.f);
  //    ekf.Correct(z);
  //    Eigen::Vector4d state = ekf.get_state();
  //
  //    if (i > 10) {
  //      ASSERT_TRUE(std::fabs(state[0] - x[0]) < 0.1f);
  //      ASSERT_TRUE(std::fabs(state[1] - x[1]) < 0.1f);
  //      ASSERT_TRUE(std::fabs(state[2] - speed) < 0.1f);
  //      ASSERT_TRUE(std::fabs(state[3] - theta) < 0.1f);
  //    }
  //  }
}

TEST(MeanFilterTest, mean_filter_test) {
  MeanFilter mean_filter;
  int window_size = 5;
  mean_filter.SetWindow(window_size);

  Eigen::Vector2d x;
  Eigen::Matrix2d var;
  x << 1.0f, 2.0f;
  mean_filter.AddMeasure(x);

  Eigen::Vector2d state = mean_filter.get_state();
  ASSERT_TRUE(std::fabs(state[0] - 1.0f) < 1e-6);
  ASSERT_TRUE(std::fabs(state[1] - 2.0f) < 1e-6);

  x << 10.0f, 200.0f;
  mean_filter.AddMeasure(x);
  state = mean_filter.get_state();
  var = mean_filter.get_variance();
  ASSERT_TRUE(std::fabs(state[0] - 5.5f) < 1e-6);
  ASSERT_TRUE(std::fabs(state[1] - 101.0f) < 1e-6);

  for (int i = 0; i < 2 * window_size; ++i) {
    x << 0.0f, 0.0f;
    mean_filter.AddMeasure(x);
  }
  state = mean_filter.get_state();
  ASSERT_TRUE(std::fabs(state[0]) < 1e-6);
  ASSERT_TRUE(std::fabs(state[1]) < 1e-6);
}
TEST(SimilarTest, cpu_test) {
  CosineSimilar similar;
  using base::Object;
  using base::ObjectPtr;

  ObjectPtr object1(new Object);
  object1->camera_supplement.object_feature.push_back(0.707f);
  object1->camera_supplement.object_feature.push_back(0.707f);
  ObjectPtr object2(new Object);
  object2->camera_supplement.object_feature.push_back(0.707f);
  object2->camera_supplement.object_feature.push_back(-0.707f);
  ObjectPtr object3(new Object);
  object3->camera_supplement.object_feature.push_back(0.707f);
  object3->camera_supplement.object_feature.push_back(0.707f);
  CameraFrame frame1;
  frame1.frame_id = 1;
  frame1.detected_objects.push_back(object1);
  frame1.detected_objects.push_back(object2);
  CameraFrame frame2;
  frame2.frame_id = 2;
  frame2.detected_objects.push_back(object3);
  base::Blob<float> sim;
  similar.Calc(&frame1, &frame2, &sim);
  const float *sim_data = sim.cpu_data();
  ASSERT_TRUE(fabs(sim_data[0] - 1) < 1e-3f);    // NOLINT
  ASSERT_TRUE(fabs(sim_data[1] - 0.0) < 1e-3f);  // NOLINT

  frame1.detected_objects.clear();
  frame2.detected_objects.clear();
  ASSERT_FALSE(similar.Calc(&frame1, &frame2, &sim));
}

TEST(SimilarTest, GPU_test) {
  inference::CudaUtil::set_device_id(0);
  GPUSimilar similar;
  using base::Object;
  using base::ObjectPtr;

  std::vector<int> shape(2);
  const float *sim_data;
  ObjectPtr object(new Object);
  CameraFrame frame1;
  CameraFrame frame2;
  std::shared_ptr<base::Blob<float>> sim1(new base::Blob<float>);
  ASSERT_FALSE(similar.Calc(&frame1, &frame1, sim1.get()));
  frame1.detected_objects.push_back(object);
  ASSERT_FALSE(similar.Calc(&frame1, &frame2, sim1.get()));
  ASSERT_FALSE(similar.Calc(&frame2, &frame1, sim1.get()));
  shape[0] = 1;
  shape[1] = 2;
  ASSERT_FALSE(similar.Calc(&frame1, &frame1, sim1.get()));
  frame1.track_feature_blob.reset(new base::Blob<float>);
  frame1.track_feature_blob->Reshape(shape);
  ASSERT_TRUE(frame1.track_feature_blob->shape(0) == 1);  // NOLINT
  ASSERT_TRUE(frame1.track_feature_blob->shape(1) == 2);  // NOLINT
  float *feature1 = frame1.track_feature_blob->mutable_cpu_data();
  feature1[0] = 0.707f;
  feature1[1] = 0.707f;

  similar.Calc(&frame1, &frame1, sim1.get());
  sim_data = sim1->cpu_data();
  ASSERT_TRUE(sim1->shape(0) == 1);           // NOLINT
  ASSERT_TRUE(sim1->shape(1) == 1);           // NOLINT
  ASSERT_TRUE(fabs(sim_data[0] - 1) < 1e-3);  // NOLINT

  frame2.detected_objects.push_back(object);
  frame2.detected_objects.push_back(object);
  frame2.detected_objects.push_back(object);
  shape[0] = 3;
  frame2.track_feature_blob.reset(new base::Blob<float>);
  frame2.track_feature_blob->Reshape(shape);
  ASSERT_TRUE(frame2.track_feature_blob->shape(0) == 3);  // NOLINT
  ASSERT_TRUE(frame2.track_feature_blob->shape(1) == 2);  // NOLINT
  float *feature2 = frame2.track_feature_blob->mutable_cpu_data();
  feature2[0] = 0.707f;
  feature2[1] = 0.707f;
  feature2[2] = 1.0f;
  feature2[3] = 0.0f;
  feature2[4] = -0.707f;
  feature2[5] = 0.707f;
  std::shared_ptr<base::Blob<float>> sim2(new base::Blob<float>);
  similar.Calc(&frame1, &frame2, sim2.get());
  ASSERT_TRUE(sim2->shape(0) == 1);  // NOLINT
  ASSERT_TRUE(sim2->shape(1) == 3);  // NOLINT
  sim_data = sim2->cpu_data();

  ASSERT_TRUE(fabs(sim_data[0] - 1) < 1e-3);            // NOLINT
  ASSERT_TRUE(fabs(sim_data[1] - sqrt(2) / 2) < 1e-3);  // NOLINT
  ASSERT_TRUE(fabs(sim_data[2] - 0) < 1e-3);            // NOLINT
}

TEST(AngleTest, angle_test) {
  HalfCircleAngle angle;
  angle.SetDirection(1.5f);
  ASSERT_TRUE(Equal(1.5f, angle.value(), 0.01f));
  angle = 1.0f;
  ASSERT_TRUE(Equal(1.0f, angle.value()));
  ASSERT_TRUE(angle == 1.0f);
  angle = angle * 2.0f;
  ASSERT_TRUE(Equal(2.0f, angle.value()));
  angle = 0.6f * 3.14f;
  angle.SetDirection(0);
  ASSERT_TRUE(Equal(-0.4f * 3.14f, angle.value(), 0.01f));
  angle.SetDirection(0);
  ASSERT_TRUE(Equal(-0.4f * 3.14f, angle.value(), 0.01f));
  angle.SetDirection(0.5f * 3.14f);
  ASSERT_TRUE(Equal(0.6f * 3.14f, angle.value(), 0.01f));
}

TEST(KalmanConstTest, const_filter_test) {
  {
    KalmanFilterConstState<1> filter;
    KalmanFilterConstState<1>::VectorNd param;
    param << 0.0;
    EXPECT_TRUE(filter.Init(param));
    filter.measure_noise_ << 10;
    filter.process_noise_ << 0.1;
    std::srand(static_cast<unsigned int>(std::time(NULL)));
    std::vector<double> measurements = {
        1.27201411, -0.27051547, -2.50816196, 0.05856895,  -3.14114835,
        1.3873894,  0.90496925,  -0.00598577, 0.48878823,  0.75704055,
        0.19266443, 2.05814734,  0.44203236,  -0.36923239, -2.74245158,
        1.71922351, 0.50960368,  -1.24154697, -1.7048239,  0.80218156};

    for (size_t i = 0; i < measurements.size(); ++i) {
      param << measurements[i];
      filter.Predict(1.0);
      filter.Correct(param);

      auto state = filter.get_state();
      EXPECT_LT(std::fabs(state(0, 0)), 1.0);
    }
  }

  {
    KalmanFilterConstState<1> filter;
    KalmanFilterConstState<1>::VectorNd param;
    param << 0.0;
    EXPECT_FALSE(filter.Predict(1.0));
    filter.Correct(param);
    EXPECT_LT((filter.get_state() - param).norm(), 1e-4);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
