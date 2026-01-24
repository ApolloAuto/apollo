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
#include "modules/v2x/fusion/libs/fusion/fusion.h"

#include <fstream>
#include <iostream>

#include "gtest/gtest.h"

#include "modules/v2x/fusion/libs/fusion/test_tools.h"

namespace apollo {
namespace v2x {
namespace ft {

TEST(Fusion, get_km_result) {
  Fusion fusion;
  std::vector<base::Object> fused_objects;
  std::vector<std::vector<base::Object>> fusion_result;
  std::vector<base::Object> objects1;
  std::vector<base::Object> objects2;
  std::vector<base::Object> objects3;
  std::vector<base::Object> objects4;
  EXPECT_FALSE(
      fusion.CombineNewResource(objects1, &fused_objects, &fusion_result));
  LoadData("/apollo/modules/v2x/fusion/test_data/fusion_object1", &objects1,
           "camera1");
  LoadData("/apollo/modules/v2x/fusion/test_data/fusion_object2", &objects2,
           "camera2");
  LoadData("/apollo/modules/v2x/fusion/test_data/fusion_object3", &objects3,
           "camera3");
  LoadData("/apollo/modules/v2x/fusion/test_data/fusion_object4", &objects4,
           "camera4");
  EXPECT_TRUE(
      fusion.CombineNewResource(objects1, &fused_objects, &fusion_result));
  EXPECT_TRUE(
      fusion.CombineNewResource(objects2, &fused_objects, &fusion_result));
  EXPECT_TRUE(
      fusion.CombineNewResource(objects3, &fused_objects, &fusion_result));
  EXPECT_TRUE(
      fusion.CombineNewResource(objects4, &fused_objects, &fusion_result));
  EXPECT_GE(fused_objects.size(), 9);
  EXPECT_LE(fused_objects.size(), 12);
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
