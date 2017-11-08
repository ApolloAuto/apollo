// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
// @date 2016/06/23 17:54:39
#include "modules/perception/traffic_light/base/test/test_define.hpp"

DEFINE_bool(enable_show_image_on_unittest,
false, "show images when test images equal.");
DEFINE_bool(enable_show_mat_on_unittest,
false, "show mat difference when test failed.");

namespace apollo {
namespace perception {
namespace traffic_light {
namespace test {

bool s_mat_equal(const cv::Mat &m1, const cv::Mat &m2) {

  if (m1.channels() != m2.channels()) {
    AERROR << "single_channel_mat_equal return false (m1.channels:"
           << m1.channels() << " != m2.channels:" << m2.channels() << ")";
    return false;
  }

  if (m1.cols != m2.cols || m1.rows != m2.rows || m1.dims != m2.dims) {
    AERROR << "single_channel_mat_equal return false. (m1.shape:"
           << m1.rows << "," << m1.cols << "," << m1.dims << ") !="
           << "(m2.channels:" << m2.rows << "," << m2.cols << "," << m2.dims << ")";
    return false;
  }

  cv::Mat diff;
  cv::compare(m1, m2, diff, cv::CMP_NE);

  int non_zero_count = cv::countNonZero(diff);
  if (0 != non_zero_count) {
    AERROR << "single_channel_mat_equal return false. " << "zero_count: "
           << non_zero_count;
    AERROR << "m1 brief:" << mat_brief(m1, 10, 10);
    AERROR << "m2 brief:" << mat_brief(m2, 10, 10);
    if (FLAGS_enable_show_mat_on_unittest) {
      AERROR << " m1:" << m1;
      AERROR << " m2:" << m2;
    }
    return false;
  }
  return true;
}

bool mat_equal(const cv::Mat &m1, const cv::Mat &m2) {

  if (m1.channels() != m2.channels()) {
    AERROR << "mat_equal return false (m1.channels:" << m1.channels()
           << " != m2.channels:" << m2.channels() << ")";
    return false;
  }

  std::vector<cv::Mat> s_m1(3), s_m2(3);
  cv::split(m1, s_m1);
  cv::split(m2, s_m2);
  for (int i = 0; i < m1.channels(); i++) {
    if (false == s_mat_equal(s_m1[i], s_m2[i])) {
      return false;
    }
  }
  return true;
}

void show_image(const std::string &name, const cv::Mat &expect, const cv::Mat &output) {

  if (FLAGS_enable_show_image_on_unittest) {
    cv::imshow(name + "_expect", expect);
    cv::imshow(name + "_output", output);
    cv::waitKey(0);
  }
}

std::string mat_brief(const cv::Mat &mat, const int max_rows, const int max_cols) {
  if (mat.type() == CV_8UC1) {
    return _show_mat<uint8_t, 1>(mat, max_rows, max_cols);
  } else if (mat.type() == CV_8UC3) {
    return _show_mat<uint8_t, 3>(mat, max_rows, max_cols);
  } else if (mat.type() == CV_32FC3) {
    return _show_mat<float, 3>(mat, max_rows, max_cols);
  } else {
    AERROR << "UNKNOW mat type:" << mat.type();
    throw std::runtime_error("UNKNOW mat type");
  }
}

} // namespace test
} // namespace traffic_light
} // namespace perception
} // namespace apollo

