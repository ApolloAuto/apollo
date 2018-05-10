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

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "gflags/gflags.h"

DEFINE_string(test_dir, "/apollo/modules/perception/data/cnnseg_test/",
              "test data directory");
DEFINE_string(pcd_name, "uscar_12_1470770225_1470770492_1349",
              "poind cloud data name");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string test_dir(FLAGS_test_dir);
  std::string det_res_file = test_dir + FLAGS_pcd_name + "-detection.txt";
  std::ifstream f_res(det_res_file.c_str(), std::ifstream::in);

  std::string line;

  std::getline(f_res, line);
  int rows, cols;
  std::istringstream iss(line);
  iss >> rows >> cols;
  ADEBUG << "#rows = " << rows << ", #cols = " << cols;

  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(0.0));
  int grid = 0;
  while (std::getline(f_res, line)) {
    std::istringstream iss(line);
    int blue, green, red;
    iss >> blue >> green >> red;
    img.at<cv::Vec3b>(grid / cols, grid % cols) = cv::Vec3b(blue, green, red);
    ++grid;
  }
  if (grid != rows * cols) {
    return -2;
  }
  f_res.close();

  std::string res_img_file = test_dir + FLAGS_pcd_name + "-detection.png";
  if (!cv::imwrite(res_img_file, img)) {
    return -1;
  }

  google::ShutDownCommandLineFlags();
  return 0;
}
