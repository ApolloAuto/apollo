/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/v2x/fusion/libs/fusion/km.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {
namespace ft {

TEST(KMkernal, get_km_result) {
  KMkernal km_matcher;
  Eigen::MatrixXf association_mat(5, 4);
  association_mat << 50, 34, 0, 0, 10, 0, 17, 0, 0, 31, 18, 10, 0, 0, 8, 17, 0,
      0, 0, 2;
  std::vector<std::pair<int, int>> match_cps;
  EXPECT_FALSE(km_matcher.GetKMResult(association_mat, &match_cps));
  EXPECT_TRUE(
      km_matcher.GetKMResult(association_mat.transpose(), &match_cps, true));
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
