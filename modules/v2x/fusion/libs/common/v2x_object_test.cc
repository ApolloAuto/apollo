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
#include "modules/v2x/fusion/libs/common/v2x_object.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {
namespace ft {
namespace base {

TEST(Object, operators) {
  std::vector<Object> objects;
  Object object;
  object.timestamp = 4.1;
  object.track_id = 0;
  objects.push_back(object);
  object.timestamp = 3.2;
  object.track_id = 1;
  objects.push_back(object);
  object.timestamp = 2.3;
  object.track_id = 2;
  objects.push_back(object);
  object.timestamp = 1.4;
  object.track_id = 3;
  objects.push_back(object);
  sort(objects.begin(), objects.end(), std::less<Object>());
  EXPECT_EQ(objects.at(0).track_id, 3);
}

}  // namespace base
}  // namespace ft
}  // namespace v2x
}  // namespace apollo
