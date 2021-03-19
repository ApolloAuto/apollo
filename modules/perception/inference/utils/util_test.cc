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

#include "modules/perception/inference/utils/util.h"

#include "gtest/gtest.h"

TEST(loadTest, test) {
  ACHECK(!apollo::perception::inference::load_binary_data("unknown.txt"));
}

TEST(UtilTest, test) {
  // std::vector<std::string> file_list =
  // {"ARZ034_12_1499218335_1499218635_500",
  //                                       "ARZ034_12_1499218335_1499218635_520",
  //                                       "ARZ034_12_1499218335_1499218635_540",
  //                                       "ARZ034_12_1499218335_1499218635_560",
  //                                       "ARZ034_12_1499218335_1499218635_580",
  //                                       "ARZ034_12_1499218335_1499218635_606",
  //                                       "ARZ034_12_1499218335_1499218635_834",
  //                                       "ARZ034_12_1499218335_1499218635_854",
  //                                       "ARZ034_12_1499218335_1499218635_874",
  //                                       "ARZ034_12_1499218335_1499218635_894"};

  // std::vector<std::string> gray_file_list = {
  //     "KL000_15_1517627412_1517627472_313"};

  // std::string image_root =
  //     "modules/perception/inference/inference_test_data/images/";
  // std::string image_ext = ".jpg";
  // int h_small = 576;
  // int w_small = 1440;
  // // int h_large = 768;
  // // int w_large = 1920;
  // int roi_h = 300;
  // int roi_w = 500;
  // int roi_x = 400;
  // int roi_y = 312;

  // // bgr yolo detector resize test
  // {
  //   int rgb_channel = 3;

  //   std::shared_ptr<apollo::perception::base::Blob<float>> blob;
  //   blob.reset(new apollo::perception::base::Blob<float>(1, h_small, w_small,
  //                                                        rgb_channel));
  //   std::shared_ptr<apollo::perception::base::SyncedMemory> src_gpu;
  //   src_gpu.reset(new apollo::perception::base::SyncedMemory(
  //       1080 * 1920 * rgb_channel * sizeof(unsigned char), true));

  //   for (auto image_file : file_list) {
  //     cv::Mat img = cv::imread(image_root + image_file + image_ext);
  //     cv::Mat img_roi;
  //     cv::Rect roi(0, roi_y, img.cols, img.rows - roi_y);
  //     img_roi.create(img.rows, img.cols, CV_8UC3);
  //     img.copyTo(img_roi);
  //     img_roi = img_roi(roi);

  //     cv::Mat img_small;
  //     cv::resize(img_roi, img_small, cv::Size(w_small, h_small));
  //     src_gpu->set_cpu_data(img_roi.data);

  //     ACHECK(apollo::perception::inference::resize(
  //         img_roi.channels(), img_roi.rows, img_roi.cols,
  //         img_roi.step1(0) / img_roi.step1(1), blob, src_gpu, 0));
  //     for (int i = 0;
  //          i < img_small.rows * img_small.cols * img_small.channels(); i++) {
  //       EXPECT_NEAR(img_small.data[i], blob->cpu_data()[i], 1) << " " << i;
  //     }
  //   }
  // }

  // // gray yolo detector resize test
  // {
  //   int gray_channel = 1;
  //   std::shared_ptr<apollo::perception::base::Blob<float>> blob;
  //   blob.reset(new apollo::perception::base::Blob<float>(1, h_small, w_small,
  //                                                        gray_channel));
  //   std::shared_ptr<apollo::perception::base::SyncedMemory> src_gpu;
  //   src_gpu.reset(new apollo::perception::base::SyncedMemory(
  //       712 * 1193 * 1 * sizeof(unsigned char), true));

  //   for (auto image_file : gray_file_list) {
  //     cv::Mat img = cv::imread(image_root + image_file + image_ext, CV_8UC1);
  //     cv::Mat img_roi;
  //     cv::Rect roi(0, roi_y, img.cols, img.rows - roi_y);
  //     img_roi.create(img.rows, img.cols, CV_8UC1);
  //     img.copyTo(img_roi);
  //     img_roi = img_roi(roi);
  //     src_gpu->set_cpu_data(img_roi.data);

  //     cv::Mat img_small;
  //     cv::resize(img_roi, img_small, cv::Size(w_small, h_small));
  //     ACHECK(apollo::perception::inference::resize(
  //         img_roi.channels(), img_roi.rows, img_roi.cols,
  //         img_roi.step1(0) / img_roi.step1(1), blob, src_gpu, 0));
  //     for (int i = 0;
  //          i < img_small.rows * img_small.cols * img_small.channels(); i++) {
  //       EXPECT_NEAR(img_small.data[i], blob->cpu_data()[i], 1) << " " << i;
  //     }
  //   }
  // }

  // // roi resize
  // {
  //   int rgb_channel = 3;
  //   std::shared_ptr<apollo::perception::base::Blob<float>> blob;
  //   blob.reset(new apollo::perception::base::Blob<float>(1, h_small, w_small,
  //                                                        rgb_channel));
  //   std::shared_ptr<apollo::perception::base::SyncedMemory> src_gpu;
  //   src_gpu.reset(new apollo::perception::base::SyncedMemory(
  //       1080 * 1920 * 3 * sizeof(unsigned char), true));

  //   for (auto image_file : file_list) {
  //     cv::Mat img = cv::imread(image_root + image_file + image_ext);
  //     cv::Mat img_roi;
  //     cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
  //     img_roi.create(img.rows, img.cols, CV_8UC3);
  //     img.copyTo(img_roi);
  //     img_roi = img_roi(roi);
  //     src_gpu->set_cpu_data(img_roi.data);

  //     cv::Mat img_small;
  //     cv::resize(img_roi, img_small, cv::Size(w_small, h_small));
  //     ACHECK(apollo::perception::inference::resize(
  //         img_roi.channels(), img_roi.rows, img_roi.cols,
  //         img_roi.step1(0) / img_roi.step1(1), blob, src_gpu, 0));
  //     for (int i = 0;
  //          i < img_small.rows * img_small.cols * img_small.channels(); i++) {
  //       EXPECT_NEAR(img_small.data[i], blob->cpu_data()[i], 1) << " " << i;
  //     }
  //   }
  // }
}
