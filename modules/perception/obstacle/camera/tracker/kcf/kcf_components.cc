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
 /******************************************************************************
  * Copyright (c) 2015, Joao Faro
  * All rights reserved.
  *
  * * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * * Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.
  *
  * * Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  *
  * Neither the name of KCFcpp nor the names of its
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *****************************************************************************/

// Components for running KCF tracker, ex: get features, detect, train in KCF

#include "modules/perception/obstacle/camera/tracker/kcf/kcf_components.h"

namespace apollo {
namespace perception {

bool KCFComponents::Init() {
  y_f_ = CreateGaussianPeak(kWindowSize_, kWindowSize_);
  cos_window_ = CalculateHann(y_f_.size());

  return true;
}

bool KCFComponents::GetFeatures(const cv::Mat &img, const cv::Rect &box,
                                std::vector<cv::Mat> *feature) {
  // Fixed image patch size
  cv::Mat box_img = img(box);
  cv::resize(box_img, box_img, cv::Size(kWindowSize_, kWindowSize_));

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
    feat[i] = feat[i].mul(cos_window_);
  }

  // Discrete Fourier Transform
  feature->clear();
  feature->resize(feat.size());
  for (size_t i = 0; i < feat.size(); ++i) {
    cv::dft(feat[i], (*feature)[i], cv::DFT_COMPLEX_OUTPUT);
  }

  return true;
}

bool KCFComponents::Detect(const Tracked &tracked_obj,
                           const std::vector<cv::Mat> &z_f, float *score) {
  cv::Mat k_f = GaussianCorrelation(z_f, tracked_obj.x_f_);

  cv::Mat response;
  cv::idft(ComplexMultiplication(tracked_obj.alpha_f_, k_f), response,
           cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

  cv::Point max_loc;
  double max_val = 0.0;
  cv::minMaxLoc(response, NULL, &max_val, NULL, &max_loc);
  *score = static_cast<float>(max_val);

  return true;
}

bool KCFComponents::Train(const cv::Mat &img, Tracked *tracked_obj) {
  cv::Mat k_f = GaussianCorrelation(tracked_obj->x_f_, tracked_obj->x_f_);

  cv::Mat alpha_f = ComplexDivision(y_f_, k_f + cv::Scalar(kLambda_, 0));
  alpha_f.copyTo(tracked_obj->alpha_f_);

  return true;
}

cv::Mat KCFComponents::GaussianCorrelation(const std::vector<cv::Mat> &xf,
                                           const std::vector<cv::Mat> &yf) {
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
  cv::exp((-1 / (kKernelSigma_ * kKernelSigma_)) *
              max(0.0, (xx + yy - 2 * xy) / numel_xf),
          k);
  k.convertTo(k, CV_32FC1);

  cv::Mat kf;
  dft(k, kf, cv::DFT_COMPLEX_OUTPUT);
  return kf;
}

cv::Mat KCFComponents::ComplexMultiplication(const cv::Mat &x1,
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

cv::Mat KCFComponents::ComplexDivision(const cv::Mat &x1, const cv::Mat &x2) {
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

cv::Mat KCFComponents::CreateGaussianPeak(const int &sizey, const int &sizex) {
  cv::Mat_<float> res(sizey, sizex);

  int syh = (sizey) / 2;
  int sxh = (sizex) / 2;

  // Assume the tracking padding is still 2.5, which means the center of box is
  // the focus
  float output_sigma =
      std::sqrt(static_cast<float>(sizex) * static_cast<float>(sizey)) / 2.5f *
      0.125f;
  float mult = -0.5 / (output_sigma * output_sigma);

  for (int i = 0; i < sizey; i++) {
    for (int j = 0; j < sizex; j++) {
      int ih = i - syh;
      int jh = j - sxh;
      res(i, j) = std::exp(mult * static_cast<float>(ih * ih + jh * jh));
    }
  }
  return FFTD(res);
}

cv::Mat KCFComponents::FFTD(cv::Mat img) {
  if (img.channels() == 1) {
    cv::Mat planes[] = {cv::Mat_<float>(img),
                        cv::Mat_<float>::zeros(img.size())};
    cv::merge(planes, 2, img);
  }

  cv::dft(img, img, 0);

  return img;
}

cv::Mat KCFComponents::CalculateHann(const cv::Size &sz) {
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
