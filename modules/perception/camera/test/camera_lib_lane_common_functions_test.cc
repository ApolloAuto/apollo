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
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/lane/common/common_functions.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(CommonFunctions, poly_fit_error_test) {
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> *coeff = nullptr;
    std::vector<Eigen::Matrix<float, 2, 1> > pos_vec;
    int order = max_poly_order;
    EXPECT_FALSE(PolyFit(pos_vec, order, coeff));
  }
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> *coeff = nullptr;
    std::vector<Eigen::Matrix<float, 2, 1> > pos_vec;
    int order = max_poly_order + 1;
    EXPECT_FALSE(PolyFit(pos_vec, order, coeff));
  }
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> coeff;
    std::vector<Eigen::Matrix<float, 2, 1> > pos_vec;
    int order = max_poly_order;
    for (int i = 0; i < order; i++) {
      Eigen::Matrix<float, 2, 1> pos;
      pos << static_cast<float>(i), static_cast<float>(i) + 1.0f;
      pos_vec.push_back(pos);
    }
    EXPECT_FALSE(PolyFit(pos_vec, order, &coeff));
  }
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> coeff;
    std::vector<Eigen::Matrix<float, 2, 1> > pos_vec;
    int order = max_poly_order;
    int n = 100;
    for (int i = 0; i < n; i++) {
      Eigen::Matrix<float, 2, 1> pos;
      pos << static_cast<float>(i),
          static_cast<float>(pow(i, 3) + pow(i, 2) + i);
      pos_vec.push_back(pos);
    }
    EXPECT_TRUE(PolyFit(pos_vec, order, &coeff, true));
    EXPECT_TRUE(PolyFit(pos_vec, order, &coeff, false));
  }
}
TEST(CommonFunctions, poly_eval_test) {
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> coeff;
    int order = max_poly_order + 2;
    float x = 1.0f;
    float y = 0.0f;
    coeff(3, 0) = 1.0f;
    coeff(2, 0) = 1.0f;
    coeff(1, 0) = 1.0f;
    coeff(0, 0) = 1.0f;
    EXPECT_FALSE(PolyEval(x, order, coeff, &y));
  }
  {
    Eigen::Matrix<float, max_poly_order + 1, 1> coeff;
    int order = max_poly_order;
    float x = 1.0f;
    float y = 0.0f;
    coeff(3, 0) = 1.0f;
    coeff(2, 0) = 1.0f;
    coeff(1, 0) = 1.0f;
    coeff(0, 0) = 1.0f;
    EXPECT_TRUE(PolyEval(x, order, coeff, &y));
  }
}
TEST(CommonFunctions, find_cc_invalid_test) {
  {
    std::vector<unsigned char> src;
    int width = 0;
    int height = 0;
    base::RectI roi;
    std::vector<ConnectedComponent> cc;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));
  }
  {
    std::vector<unsigned char> src;
    int width = 100;
    int height = 100;
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        if (i % 5 == 0 || j % 5 == 0) {
          src.push_back(1);
        } else {
          src.push_back(0);
        }
      }
    }
    std::vector<ConnectedComponent> cc;
    base::RectI roi;
    roi.x = -1;
    roi.y = 0;
    roi.width = width;
    roi.height = height;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));

    roi.x = 0;
    roi.y = -1;
    roi.width = width;
    roi.height = height;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));

    roi.x = 0;
    roi.y = 0;
    roi.width = width + 5;
    roi.height = height;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));

    roi.x = 0;
    roi.y = 0;
    roi.width = width;
    roi.height = height + 5;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));

    roi.x = 0;
    roi.y = 0;
    roi.width = 1;
    roi.height = 1;
    EXPECT_FALSE(FindCC(src, width, height, roi, &cc));
  }
}
TEST(CommonFunctions, find_cc_test) {
  std::vector<unsigned char> src;
  std::vector<ConnectedComponent> cc;
  std::string impath =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/common/data/test_cc.png";
  cv::Mat img = cv::imread(impath);
  int width = img.cols;
  int height = img.rows;
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      src.push_back(img.at<uchar>(i, j));
    }
  }
  base::RectI roi;
  roi.x = 0;
  roi.y = 0;
  roi.width = width;
  roi.height = height;
  EXPECT_TRUE(FindCC(src, width, height, roi, &cc));
}

TEST(CommonFunctions, QuickSortTest) {
  int nsize = 10;
  std::vector<float> value_vec;
  std::string msg = "input value: ";
  char tmp[256];
  unsigned int seed = 0;
  for (int i = 0; i < nsize; i++) {
    int rand_value = rand_r(&seed) % 100;
    float value = static_cast<float>(rand_value / 100 + rand_value);
    value_vec.push_back(value);
    snprintf(tmp, sizeof(tmp), "[%d %f] ", i, value);
    msg += tmp;
  }
  AINFO << msg;
  std::vector<int> index(nsize);
  QuickSort(&(index[0]), &(value_vec[0]), nsize);
  msg = "sort value: ";
  for (int i = 0; i < nsize; i++) {
    snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
    msg += tmp;
  }
  AINFO << msg;
}
TEST(CommonFunctions, FindKSmallValueTest) {
  int nsize = 10;
  std::vector<float> value_vec;
  std::string msg = "input value: ";
  char tmp[256];
  unsigned int seed = 0;
  for (int i = 0; i < nsize; i++) {
    int rand_value = rand_r(&seed) % 10;
    float value =
        0.32f * static_cast<float>(rand_value) + static_cast<float>(rand_value);
    value_vec.push_back(value);
    snprintf(tmp, sizeof(tmp), "[%d %f] ", i, value);
    msg += tmp;
  }
  AINFO << msg;
  {
    int k = nsize + 1;
    std::vector<int> index(k);
    EXPECT_FALSE(FindKSmallValue(&(value_vec[0]), nsize, k, &(index[0])));
    EXPECT_FALSE(FindKSmallValue(&(value_vec[0]), nsize, 0, &(index[0])));
  }
  {
    int k = 2;
    std::vector<int> index(k);
    FindKSmallValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KSmallValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    int k = 5;
    std::vector<int> index(k);
    FindKSmallValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KSmallValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    int k = nsize;
    std::vector<int> index(k);
    FindKSmallValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KSmallValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    value_vec[0] = -2;
    value_vec[1] = 3;
    value_vec[2] = -1;
    value_vec[3] = 0;
    value_vec[4] = 23;
    value_vec[5] = 33;
    value_vec[6] = 34;
    value_vec[7] = 5;
    value_vec[8] = -3;
    value_vec[9] = 9;
    int k = 2;
    std::vector<int> index(k);
    FindKSmallValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KSmallValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
}

TEST(CommonFunctions, FindKLargeValueTest) {
  int nsize = 10;
  std::vector<float> value_vec;
  std::string msg = "input value: ";
  char tmp[256];
  unsigned int seed = 5;
  for (int i = 0; i < nsize; i++) {
    int rand_value = rand_r(&seed) % 10;
    float value =
        0.2f * static_cast<float>(rand_value) + static_cast<float>(rand_value);
    value_vec.push_back(value);
    snprintf(tmp, sizeof(tmp), "[%d %f] ", i, value);
    msg += tmp;
  }
  AINFO << msg;
  {
    int k = nsize + 1;
    std::vector<int> index(k);
    EXPECT_FALSE(FindKLargeValue(&(value_vec[0]), nsize, k, &(index[0])));
    EXPECT_FALSE(FindKLargeValue(&(value_vec[0]), nsize, 0, &(index[0])));
  }
  {
    int k = 2;
    std::vector<int> index(k);
    FindKLargeValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KLargeValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    int k = 5;
    std::vector<int> index(k);
    FindKLargeValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KLargeValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    int k = nsize;
    std::vector<int> index(k);
    FindKLargeValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KLargeValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
  {
    value_vec[0] = -2;
    value_vec[1] = 3;
    value_vec[2] = -1;
    value_vec[3] = 0;
    value_vec[4] = 23;
    value_vec[5] = 33;
    value_vec[6] = 34;
    value_vec[7] = 5;
    value_vec[8] = -3;
    value_vec[9] = 9;
    int k = 2;
    std::vector<int> index(k);
    FindKLargeValue(&(value_vec[0]), nsize, k, &(index[0]));
    msg = "KLargeValue: ";
    for (int i = 0; i < k; i++) {
      snprintf(tmp, sizeof(tmp), "[%d %f] ", index[i], value_vec[index[i]]);
      msg += tmp;
    }
    AINFO << msg;
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
