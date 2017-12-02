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

#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"

#include "Eigen/Core"
#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

TEST(HungarianOptimizerTest, maximize) {
  // case 1
  Eigen::MatrixXf association_mat_1(3, 4);
  association_mat_1 << 4.7, 3.8, 1.0, 2.0, 4.1, 3.0, 2.0, -3.0, 1.0, 2.0, 4.7,
      4.9;
  std::vector<std::vector<double>> cost(association_mat_1.rows());
  for (int i = 0; i < association_mat_1.rows(); i++) {
    cost[i].resize(association_mat_1.cols());
    for (int j = 0; j < association_mat_1.cols(); j++) {
      cost[i][j] = association_mat_1(i, j);
    }
  }
  HungarianOptimizer hungarian_optimizer_1(cost);
  std::vector<int> agent;
  std::vector<int> task;
  hungarian_optimizer_1.maximize(&agent, &task);
  EXPECT_EQ(0, agent[0]);
  EXPECT_EQ(1, agent[1]);
  EXPECT_EQ(2, agent[2]);
  EXPECT_EQ(1, task[0]);
  EXPECT_EQ(0, task[1]);
  EXPECT_EQ(3, task[2]);

  // case 2
  Eigen::MatrixXf association_mat_2(2, 2);
  association_mat_2 << 0.0, 0.0, 0.0, 0.0;
  cost.resize(association_mat_2.rows());
  for (int i = 0; i < association_mat_2.rows(); i++) {
    cost[i].resize(association_mat_2.cols());
    for (int j = 0; j < association_mat_2.cols(); j++) {
      cost[i][j] = association_mat_2(i, j);
    }
  }
  HungarianOptimizer hungarian_optimizer_2(cost);
  agent.clear();
  task.clear();
  hungarian_optimizer_2.maximize(&agent, &task);
  EXPECT_EQ(0, agent[0]);
  EXPECT_EQ(1, agent[1]);
  EXPECT_EQ(0, task[0]);
  EXPECT_EQ(1, task[1]);

  // case 3
  Eigen::MatrixXf association_mat_3(2, 2);
  association_mat_3 << 3.0, 3.0, 3.0, 3.0;
  cost.resize(association_mat_3.rows());
  for (int i = 0; i < association_mat_3.rows(); i++) {
    cost[i].resize(association_mat_3.cols());
    for (int j = 0; j < association_mat_3.cols(); j++) {
      cost[i][j] = association_mat_3(i, j);
    }
  }
  HungarianOptimizer hungarian_optimizer_3(cost);
  agent.clear();
  task.clear();
  hungarian_optimizer_3.maximize(&agent, &task);
  EXPECT_EQ(0, agent[0]);
  EXPECT_EQ(1, agent[1]);
  EXPECT_EQ(0, task[0]);
  EXPECT_EQ(1, task[1]);

  // case 4
  for (size_t i = 0; i < cost.size(); ++i) {
    cost[i].clear();
  }
  cost.clear();
  HungarianOptimizer hungarian_optimizer_4(cost);
  agent.clear();
  task.clear();
  hungarian_optimizer_4.maximize(&agent, &task);
  EXPECT_EQ(0, agent.size());
  EXPECT_EQ(0, task.size());
}

TEST(HungarianOptimizerTest, minimize) {
  Eigen::MatrixXf association_mat(3, 4);
  association_mat << 0.3, 1.2, 4.0, 3.0, 0.9, 2.0, 3.0, 8.0, 4.0, 3.0, 0.3, 0.1;
  std::vector<std::vector<double>> cost(association_mat.rows());
  for (int i = 0; i < association_mat.rows(); i++) {
    cost[i].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); j++) {
      cost[i][j] = association_mat(i, j);
    }
  }
  HungarianOptimizer* hungarian_optimizer = new HungarianOptimizer(cost);
  std::vector<int> agent;
  std::vector<int> task;
  hungarian_optimizer->minimize(&agent, &task);
  EXPECT_EQ(0, agent[0]);
  EXPECT_EQ(1, agent[1]);
  EXPECT_EQ(2, agent[2]);
  EXPECT_EQ(1, task[0]);
  EXPECT_EQ(0, task[1]);
  EXPECT_EQ(3, task[2]);
  if (hungarian_optimizer != nullptr) {
    delete hungarian_optimizer;
    hungarian_optimizer = nullptr;
  }
}

}  // namespace perception
}  // namespace apollo
