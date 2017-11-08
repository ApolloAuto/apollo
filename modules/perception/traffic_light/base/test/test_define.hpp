// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/10/08 13:54:43
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_TEST_TEST_DEFINE_HPP
#define ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_TEST_TEST_DEFINE_HPP

#include <string>
#include <sstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "modules/common/log.h"
#include <gflags/gflags.h>

DECLARE_bool(enable_show_image_on_unittest);

namespace apollo {
namespace perception {
namespace traffic_light {
namespace test {

//@brief v1 approximately equals to v2
//@return true/false
template<typename T>
bool approximately_equal(const T &v1, const T &v2, const T &error) {
  if (std::abs(1 - v1 / v2) < error) {
    return true;
  } else {
    AERROR << "v1:" << v1 << " != " << "v2:" << v2 << " error:" << error;
    return false;
  }
}

//@brief show mat brief
template<typename T, int N>
std::string _show_mat(const cv::Mat &mat, const int max_rows, const int max_cols) {
  std::stringstream ss;
  ss << "[" << mat.rows << "X" << mat.cols << "X" << mat.channels() << "]" << std::endl;
  int i = 0;
  int j = 0;
  for (i = 0; i < mat.rows && i < max_rows; i++) {
    for (j = 0; j < mat.cols && j < max_cols; j++) {
      cv::Vec<T, N> ele = mat.at<cv::Vec<T, N> >(i, j);
      ss << ele;
    }
    if (j < mat.cols) {
      ss << "...";
    }
    ss << "\n";
  }
  ss << "\n";
  if (i < mat.rows) {
    ss << "...\n";
  }
  return ss.str();
}

template<typename T, int N>
std::stringstream &operator<<(std::stringstream &ss, const cv::Vec<T, N> ele) {
  ss << "(";
  for (int k = 0; k < N; k++) {
    ss << (unsigned int) ele(k);
    if (k != N - 1) {
      ss << ",";
    }
  }
  ss << ")";
  return ss;
}

template<int N>
std::stringstream &operator<<(std::stringstream &ss, const cv::Vec<float, N> ele) {
  ss << "(";
  for (int k = 0; k < N; k++) {
    ss << (float) ele(k);
    if (k != N - 1) {
      ss << ",";
    }
  }
  ss << ")";
  return ss;
}

//@brief show mat brief
//@return string
std::string mat_brief(const cv::Mat &mat, const int max_rows, const int max_cols);

//@brief test m1 == m2
//@param m1 Single Channel mat
//@param m2 Single channel mat
//@return true/false
bool s_mat_equal(const cv::Mat &m1, const cv::Mat &m2);

//@brief test m1 == m2
//@param normal mat 
//@param normal mat
//@return true/false
bool mat_equal(const cv::Mat &m1, const cv::Mat &m2);

//@brief show images on screen
//       when set gflags false, it will do nothing.
void show_image(const std::string &name, const cv::Mat &expect, const cv::Mat &output);

} // namespace test
} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif //ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_TEST_TEST_DEFINE_HPP
