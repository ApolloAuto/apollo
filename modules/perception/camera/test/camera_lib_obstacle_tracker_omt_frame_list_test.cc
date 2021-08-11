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
#include "gtest/gtest.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(PatchIndicatorTest, PatchIndicator_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  ACHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  PatchIndicator p1;
  EXPECT_EQ(p1.frame_id, -1);
  EXPECT_EQ(p1.patch_id, -1);

  PatchIndicator p2(1, 1);
  EXPECT_EQ(p2.frame_id, 1);
  EXPECT_EQ(p2.patch_id, 1);

  PatchIndicator p3(1, 1);
  EXPECT_TRUE(p2 == p3);

  EXPECT_EQ(p3.to_string(), " | 1 (1)");
}

TEST(SimilarMapTest, SimilarMap_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  ACHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  SimilarMap similar_map;
  ASSERT_FALSE(similar_map.Init(0));
  ASSERT_TRUE(similar_map.Init(10));
  auto sim = similar_map.get(0, 0).get();
  sim->Reshape({2, 2});
  float *sim_data = sim->mutable_cpu_data();
  *sim_data = 1.2f;
  *(sim_data + 1) = 1.4f;
  *(sim_data + 2) = 1.6f;
  *(sim_data + 3) = 1.8f;
  auto sim1 = similar_map.get(0, 0)->cpu_data();
  EXPECT_FLOAT_EQ(*sim1, 1.2);
  EXPECT_FLOAT_EQ(*(sim1 + 1), 1.4);
  EXPECT_FLOAT_EQ(*(sim1 + 2), 1.6);
  EXPECT_FLOAT_EQ(*(sim1 + 3), 1.8);

  PatchIndicator p1(0, 0);
  PatchIndicator p2(0, 1);
  EXPECT_FLOAT_EQ(similar_map.sim(p1, p2), 1.4);

  std::shared_ptr<base::Blob<float>> set_blob(new base::Blob<float>({4, 1}));
  float *sim_data1 = set_blob->mutable_cpu_data();
  *sim_data1 = 0.2f;
  *(sim_data1 + 1) = 0.4f;
  *(sim_data1 + 2) = 0.6f;
  *(sim_data1 + 3) = 0.8f;
  similar_map.set(0, 1, set_blob);
  auto sim2 = similar_map.get(0, 1)->cpu_data();
  EXPECT_FLOAT_EQ(*sim2, 0.2);
  EXPECT_FLOAT_EQ(*(sim2 + 1), 0.4);
  EXPECT_FLOAT_EQ(*(sim2 + 2), 0.6);
  EXPECT_FLOAT_EQ(*(sim2 + 3), 0.8);
}

TEST(FrameListTest, FrameList_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  ACHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  FrameList frame_list;
  ASSERT_EQ(frame_list.Size(), 0);
  ASSERT_FALSE(frame_list.Init(-1));
  ASSERT_TRUE(frame_list.Init(5));
  ASSERT_EQ(frame_list.OldestFrameId(), 0);
  std::vector<CameraFrame> frames(6);
  for (int i = 0; i < 6; i++) {
    frames[i].frame_id = i;
    frame_list.Add(&frames[i]);
  }
  ASSERT_EQ(frame_list.Size(), 5);
  ASSERT_EQ(frame_list.OldestFrameId(), 1);
  ASSERT_EQ(frame_list.get_frame(5), frame_list.get_frame(-1));
  ASSERT_EQ(frame_list[5], frame_list[-1]);
  base::ObjectPtr object;
  frames[1].detected_objects.push_back(object);
  PatchIndicator indicator(1, 0);
  base::ObjectPtr object1 = frame_list.get_object(indicator);
  ASSERT_EQ(object1, object);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
