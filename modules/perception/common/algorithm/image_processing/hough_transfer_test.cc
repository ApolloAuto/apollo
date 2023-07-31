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

#include "modules/perception/common/algorithm/image_processing/hough_transfer.h"

namespace apollo {
namespace perception {
namespace algorithm {

TEST(HoughTransferTest, image_no_line_test) {
  int iarray[] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  std::vector<int> image(iarray, iarray + 16);
  std::vector<HoughLine> lines;
  lines.reserve(10);
  HoughTransfer transfer;
  bool flag = transfer.Init(4, 4, 1.0, 1.0);
  EXPECT_TRUE(flag);
  transfer.ImageVote(image, true);
  transfer.GetLines(4, 1, 1, true, &lines);
  EXPECT_EQ(0, lines.size());
  // with distribute false test
  transfer.ImageVote(image, false);
  transfer.GetLines(4, 1, 1, false, &lines);
  EXPECT_EQ(0, lines.size());
  // wrong size
  std::vector<int> image2(iarray, iarray + 9);
  flag = transfer.ImageVote(image2, true);
  EXPECT_FALSE(flag);
  transfer.GetLines(4, 1, 1, true, &lines);
  EXPECT_EQ(0, lines.size());
  // nullptr
  transfer.ImageVote(image, true);
  transfer.GetLines(4, 1, 1, true, nullptr);
  EXPECT_EQ(0, lines.size());
  // wrong input
  flag = transfer.Init(0, 1, 1.0, 1.0);
  EXPECT_FALSE(flag);
  flag = transfer.ImageVote(image, true);
  EXPECT_FALSE(flag);
  transfer.GetLines(4, 1, 1, true, &lines);
  EXPECT_EQ(0, lines.size());
  unsigned int size_m = transfer.MemoryConsume();
  EXPECT_EQ(size_m, 0);
  flag = transfer.Init(1, 1, 100, 760);
  EXPECT_FALSE(flag);
  flag = transfer.ImageVote(image, true);
  EXPECT_FALSE(flag);
  transfer.GetLines(4, 1, 1, true, &lines);
  EXPECT_EQ(0, lines.size());
}

TEST(HoughTransferTest, image_one_line_test) {
  int iarray[] = {0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0};
  std::vector<int> image(iarray, iarray + 16);
  std::vector<HoughLine> lines;
  lines.reserve(10);
  HoughTransfer transfer;
  bool flag = transfer.Init(4, 4, 1.0, 1.0);
  EXPECT_TRUE(flag);
  EXPECT_NEAR(transfer.get_d_r(), 1.0, 1e-6);
  EXPECT_NEAR(transfer.get_d_theta(), 1.0 * M_PI / 180, 1e-6);
  EXPECT_EQ(transfer.get_img_w(), 4);
  EXPECT_EQ(transfer.get_img_h(), 4);
  transfer.ImageVote(image, true);
  transfer.GetLines(4, 2, 4, true, &lines);
  EXPECT_GE(lines.size(), 1);
  // with distribute false test
  transfer.ImageVote(image, false);
  transfer.GetLines(4, 1, 1, false, &lines);
  EXPECT_GE(lines.size(), 1);
  unsigned int size_m = transfer.MemoryConsume();
  EXPECT_GE(size_m, 1);
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
