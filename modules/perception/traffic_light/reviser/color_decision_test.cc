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
#include "modules/perception/traffic_light/reviser/color_decision.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class DecisionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    reviser_ = new ColorReviser;
    ASSERT_TRUE(reviser_->Init());
    AINFO << "Setup";
  }
  ~DecisionTest() { delete reviser_; }

 protected:
  BaseReviser *reviser_;
};

void do_test(BaseReviser *reviser_, const std::vector<int> &color_list,
             TLColor color) {
  ReviseOption option(100);
  for (size_t i = 0; i < color_list.size(); ++i) {
    std::vector<LightPtr> light;
    light.emplace_back(new Light);
    light[0]->status.color = TLColor(color_list[i]);
    reviser_->Revise(option, &light);
    option.ts += 0.1;
    ASSERT_TRUE(light[0]->status.color == color);
  }
}

TEST_F(DecisionTest, red_flash) {
  std::vector<int> color_list = {1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1};
  do_test(reviser_, color_list, RED);
}

TEST_F(DecisionTest, green_flash) {
  std::vector<int> color_list = {3, 3, 3, 0, 3, 0, 3, 3, 0, 3,
                                 0, 3, 0, 3, 0, 0, 0, 0, 0, 3};
  do_test(reviser_, color_list, GREEN);
}

TEST_F(DecisionTest, yellow_flash) {
  std::vector<int> color_list = {2, 0, 2, 2, 2, 0, 2, 2,
                                 0, 2, 0, 0, 2, 2, 0, 2};
  do_test(reviser_, color_list, YELLOW);
}

TEST_F(DecisionTest, mix) {
  std::vector<int> color_list = {3, 3, 3, 0, 0, 0, 3, 0, 3, 1, 0, 1, 0, 1,
                                 0, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 3};
  std::vector<int> gt_list = {3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3};
  ReviseOption option(100);
  for (size_t i = 0; i < color_list.size(); ++i) {
    std::vector<LightPtr> light;
    light.emplace_back(new Light);
    light[0]->status.color = TLColor(color_list[i]);
    reviser_->Revise(option, &light);
    option.ts += 0.1;
    ASSERT_TRUE(light[0]->status.color == TLColor(gt_list[i]));
  }
}

TEST_F(DecisionTest, mix_yellow) {
  std::vector<int> color_list = {3, 3, 3, 0, 0, 0, 3, 0, 3, 2, 0, 2, 0, 0,
                                 2, 1, 0, 2, 0, 2, 0, 1, 1, 3, 0, 0, 3};
  std::vector<int> gt_list = {3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2,
                              2, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3};
  ReviseOption option(100);
  for (size_t i = 0; i < color_list.size(); ++i) {
    std::vector<LightPtr> light;
    light.emplace_back(new Light);
    light[0]->status.color = TLColor(color_list[i]);
    reviser_->Revise(option, &light);
    option.ts += 0.1;
    ASSERT_TRUE(TLColor(gt_list[i]) == light[0]->status.color);
  }
}

TEST_F(DecisionTest, mix_yellow_red_flash) {
  std::vector<double> ts_list = {
      11.784286, 11.822046, 11.860939, 11.899873, 11.938863, 11.976958,
      12.019318, 12.060082, 12.085589, 12.142431, 12.206964, 12.244730,
      12.283630, 12.343854, 12.406436, 12.447242, 12.523147, 12.590790,
      12.637774, 12.739506, 12.778026, 12.816889, 12.854136, 12.902307,
      12.941082, 12.978899, 13.052896, 13.123380, 13.162235, 13.200546,
      13.238770, 13.276381, 13.336628, 13.397770, 13.447470, 13.506018,
      13.556838, 13.595315, 13.638051, 13.758083, 13.798111, 13.833773,
      13.872687, 13.910888, 13.949736, 13.992507, 14.055734, 14.096796,
      14.154976, 14.221338, 14.259621, 14.297711, 14.361917, 14.438549,
      14.501423, 14.565565, 14.603986, 14.670650, 14.738613, 14.777124,
      14.836051, 14.928467, 14.955982, 15.007555, 15.042017, 15.100018,
      15.141860, 15.187705, 15.236564, 15.281425, 15.320009, 15.359252,
      15.398300, 15.424032, 15.461977, 15.500767, 15.538779, 15.576964,
      15.615162, 15.681964, 15.720530, 15.750545, 15.791621, 15.854720,
      15.937057, 16.015527, 16.053107, 16.092126, 16.130880, 16.168330};
  std::vector<int> color_list = {
      3, 3, 3, 3, 3, 3, 4, 1, 4, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2,
      2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 1, 4, 4, 2, 4, 2,
      4, 4, 4, 1, 1, 1, 1, 1, 1, 4, 2, 4, 1, 4, 4, 1, 1, 4, 1, 1, 1};

  std::vector<int> gt_list = {
      3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  ReviseOption option(100);
  for (size_t i = 0; i < color_list.size(); ++i) {
    std::vector<LightPtr> light;
    light.emplace_back(new Light);
    light[0]->status.color = TLColor(color_list[i]);
    reviser_->Revise(option, &light);
    option.ts = ts_list[i];
    ASSERT_TRUE(TLColor(gt_list[i]) == light[0]->status.color) << " i: " << i;
  }
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
