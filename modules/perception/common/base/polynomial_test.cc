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
#include "gtest/gtest.h"

#include "modules/perception/common/base/polynomial.h"

namespace apollo {
namespace perception {
namespace base {

TEST(BaseTest, polynomial_test) {
  // f(x) = 1 + 2 * x^2 + 3 * x^3 + 5 * x^5
  Polynomial poly;
  poly[0] = 1.0;
  poly[2] = 2.0;
  poly[3] = 3.0;
  poly[5] = 5.0;

  EXPECT_NEAR(poly[0], 1.0, 1e-8);
  EXPECT_NEAR(poly[1], 0.0, 1e-8);
  EXPECT_NEAR(poly[2], 2.0, 1e-8);
  EXPECT_NEAR(poly[3], 3.0, 1e-8);
  EXPECT_NEAR(poly[4], 0.0, 1e-8);
  EXPECT_NEAR(poly[5], 5.0, 1e-8);

  EXPECT_NEAR(poly(0.0), 1.0, 1e-6);
  EXPECT_NEAR(poly(1.0), 11.0, 1e-6);
  // f(x) = 1 + 2 * x^2 + 3 * x^3 + 5 * x^5 + 6 * x^6
  poly[6] = 6.0;
  EXPECT_NEAR(poly(1.0), 17.0, 1e-6);
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
