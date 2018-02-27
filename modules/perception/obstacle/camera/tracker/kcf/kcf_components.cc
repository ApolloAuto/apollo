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

// Components for running KCF tracker, ex: get features, detect, train in KCF

#include "modules/perception/obstacle/camera/tracker/kcf/kcf_components.h"

namespace apollo {
namespace perception {

bool KCFComponents::Init() {

  y_f_ = CreateGaussianPeak(window_size_, window_size_);
  cos_window_ = CalculateHann(y_f_.size());

  return true;
}

bool KCFComponents::GetFeatures(const cv::Mat &img, const cv::Rect &box,
                                std::vector<cv::Mat> &feature) {
  // Fixed image patch size
  cv::Mat box_img = img(box);
  cv::resize(box_img, box_img, cv::Size(_window_size, _window_size));

  // Gray scale features
  if (box_img.channels() == 3) {
    cv::cvtColor(box_img, box_img, CV_BGR2GRAY);
  }
  box_img.convertTo(box_img, CV_32FC1, 1.0 / 255);
  box_img = box_img - cv::mean(box_img).val[0];
  std::vector<cv::Mat> feat;
  feat.emplace_back(box_img);

  // Cosine Window Smoothness
  for (size_t i = 0; i < feat.size(); ++i) {
    feat[i] = feat[i].mul(_cos_window);
  }

  // Discrete Fourier Transform
  feature.clear();
  feature.resize(feat.size());
  for (size_t i = 0; i < feat.size(); ++i) {
    cv::dft(feat[i], feature[i], cv::DFT_COMPLEX_OUTPUT);
  }

  return true;
}

bool KCFComponents::Detect(const Tracked &tracked_obj,
                           const std::vector<cv::Mat> &z_f, float &score) {
  cv::Mat k_f = gaussian_correlation(z_f, tracked_obj._x_f);

  cv::Mat response;
  cv::idft(complex_multiplication(tracked_obj._alpha_f, k_f), response,
           cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

  cv::Point max_loc;
  double max_val = 0.0;
  cv::minMaxLoc(response, NULL, &max_val, NULL, &max_loc);
  score = static_cast<float>(max_val);

  //    // TODO use max_loc for position confidence
  //    max_loc.x -= z_f[0].cols / 2;
  //    max_loc.y -= z_f[0].rows / 2;
  //    // Debug visualization
  //    cv::Mat response_color;
  //    response.convertTo(response_color, CV_8UC1, 255);
  //    cv::applyColorMap(response_color, response_color, cv::COLORMAP_JET);
  //    cv::imshow("response", response_color);
  //    cv::waitKey(0);

  return true;
}

bool KCFComponents::Train(const cv::Mat &img, Tracked &tracked_obj) {
  cv::Mat k_f = gaussian_correlation(tracked_obj._x_f, tracked_obj._x_f);

  cv::Mat alpha_f = complex_division(_y_f, k_f + cv::Scalar(_lambda, 0));
  alpha_f.copyTo(tracked_obj._alpha_f);

  return true;
}

cv::Mat KCFComponents::gaussian_correlation(std::vector<cv::Mat> xf,
                                            std::vector<cv::Mat> yf) {
  int nn = xf[0].size().area();
  double xx = 0;
  double yy = 0;
  std::vector<cv::Mat> xyf_vector(xf.size());
  cv::Mat xy(xf[0].size(), CV_32FC1, cv::Scalar(0.0));
  cv::Mat xyf;
  cv::Mat xy_temp;
  for (unsigned int i = 0; i < xf.size(); ++i) {
    xx += cv::norm(xf[i]) * cv::norm(xf[i]) / nn;
    yy += cv::norm(yf[i]) * cv::norm(yf[i]) / nn;
    cv::mulSpectrums(xf[i], yf[i], xyf, 0, true);
    cv::idft(xyf, xy_temp, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    xy += xy_temp;
  }

  cv::Mat k;
  float numel_xf = nn * xf.size();
  cv::exp((-1 / (_kernel_sigma * _kernel_sigma)) *
              max(0.0, (xx + yy - 2 * xy) / numel_xf),
          k);
  k.convertTo(k, CV_32FC1);

  cv::Mat kf;
  dft(k, kf, cv::DFT_COMPLEX_OUTPUT);
  return kf;
}

cv::Mat KCFComponents::complex_multiplication(const cv::Mat &x1,
                                              const cv::Mat &x2) {
  std::vector<cv::Mat> planes1;
  cv::split(x1, planes1);

  std::vector<cv::Mat> planes2;
  cv::split(x2, planes2);

  std::vector<cv::Mat> complex(2);
  complex[0] = planes1[0].mul(planes2[0]) - planes1[1].mul(planes2[1]);
  complex[1] = planes1[0].mul(planes2[1]) + planes1[1].mul(planes2[0]);

  cv::Mat result;
  cv::merge(complex, result);
  return result;
}

cv::Mat KCFComponents::complex_division(const cv::Mat &x1, const cv::Mat &x2) {
  std::vector<cv::Mat> planes1;
  cv::split(x1, planes1);

  std::vector<cv::Mat> planes2;
  cv::split(x2, planes2);

  std::vector<cv::Mat> complex(2);
  cv::Mat cc = planes2[0].mul(planes2[0]);
  cv::Mat dd = planes2[1].mul(planes2[1]);
  complex[0] =
      (planes1[0].mul(planes2[0]) + planes1[1].mul(planes2[1])) / (cc + dd);
  complex[1] =
      (-planes1[0].mul(planes2[1]) + planes1[1].mul(planes2[0])) / (cc + dd);

  cv::Mat result;
  cv::merge(complex, result);

  return result;
}

cv::Mat KCFComponents::create_gaussian_peak(int sizey, int sizex) {
  cv::Mat_<float> res(sizey, sizex);

  int syh = (sizey) / 2;
  int sxh = (sizex) / 2;

  // Assume the tracking padding is still 2.5, which means the center of box is
  // the focus
  float output_sigma = std::sqrt((float)sizex * sizey) / 2.5 * 0.125;
  float mult = -0.5 / (output_sigma * output_sigma);

  for (int i = 0; i < sizey; i++) {
    for (int j = 0; j < sizex; j++) {
      int ih = i - syh;
      int jh = j - sxh;
      res(i, j) = std::exp(mult * (float)(ih * ih + jh * jh));
    }
  }
  return fftd(res);
}

cv::Mat KCFComponents::fftd(cv::Mat img) {
  if (img.channels() == 1) {
    cv::Mat planes[] = {cv::Mat_<float>(img),
                        cv::Mat_<float>::zeros(img.size())};
    cv::merge(planes, 2, img);
  }

  cv::dft(img, img, 0);

  return img;
}

cv::Mat KCFComponents::calculate_hann(const cv::Size &sz) {
  cv::Mat temp1(cv::Size(sz.width, 1), CV_32FC1);
  cv::Mat temp2(cv::Size(sz.height, 1), CV_32FC1);

  for (int i = 0; i < sz.width; ++i) {
    temp1.at<float>(0, i) = 0.5 * (1 - std::cos(2 * M_PI * i / (sz.width - 1)));
  }

  for (int i = 0; i < sz.height; ++i) {
    temp2.at<float>(0, i) =
        0.5 * (1 - std::cos(2 * M_PI * i / (sz.height - 1)));
  }

  return temp2.t() * temp1;
}

}  // namespace perception
}  // namespace apollo
