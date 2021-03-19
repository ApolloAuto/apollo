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

namespace apollo {
namespace perception {
namespace camera {

TEST(VehtemplateTest, GetVisualObjHwlBySearchTemplatesTest) {
  ObjectTemplateManagerInitOptions options;
  options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  options.conf_file = "object_template.pt";
  ObjectTemplateManager* manager = ObjectTemplateManager::Instance();
  EXPECT_TRUE(manager->Init(options));

  {
    float hwl[] = {2.16173f, 1.87321f, 5.11133f};
    int index = 0;
    manager->VehObjHwlBySearchTemplates(hwl, &index);
    EXPECT_EQ(index, 4);
  }

  // flip case
  {
    float hwl[] = {1.87800f, 4.18000f, 1.64000f};
    int index = 0;
    bool is_flip = false;
    manager->VehObjHwlBySearchTemplates(hwl, &index, &is_flip);
    EXPECT_EQ(index, 3);
    EXPECT_TRUE(is_flip);
  }

  {
    float hwl[] = {2.16173f, 1.87321f, 5.11133f};
    int index = 0;
    bool is_flip = false;
    float score = manager->VehObjHwlBySearchTemplates(hwl, &index, &is_flip);
    EXPECT_EQ(index, 4);
    EXPECT_FALSE(is_flip);
    EXPECT_NEAR(score, 1.0, 1e-2);
  }

  {
    float hwl[] = {2.16173f, 0.01f, 0.01f};
    int index = 0;
    bool is_flip = false;
    float score = manager->VehObjHwlBySearchTemplates(hwl, &index, &is_flip);
    EXPECT_EQ(index, 0);
    EXPECT_FALSE(is_flip);
    EXPECT_NEAR(score, -1.0, 1e-6);
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
