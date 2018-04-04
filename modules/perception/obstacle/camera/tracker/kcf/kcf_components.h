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
//
// Reference:
//
// 1. Henriques, João F., et al. "High-speed tracking with kernelized
// correlation filters."
// IEEE Transactions on Pattern Analysis and Machine Intelligence 37.3 (2015):
// 583-596.
//
// 2. KCFcpp, https://github.com/joaofaro/KCFcpp
// BSD 3-Clause "New" or "Revised" License
// https://github.com/joaofaro/KCFcpp/blob/master/LICENSE
//
// 3. KCF, https://github.com/foolwood/KCF
// Another implementation of KCF tracker

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/perception/obstacle/camera/tracker/base_affinity_tracker.h"

namespace apollo {
namespace perception {

class KCFComponents {
 public:
  KCFComponents() {}

  bool Init();

  // Get x_f or z_f
  bool GetFeatures(const cv::Mat &img, const cv::Rect &box,
                   std::vector<cv::Mat> *feature);

  // Get response score
  bool Detect(const Tracked &tracked_obj, const std::vector<cv::Mat> &z_f,
              float *score);

  // Get alpha_f
  bool Train(const cv::Mat &img, Tracked *tracked_obj);

 private:
  cv::Mat GaussianCorrelation(const std::vector<cv::Mat> &xf,
                              const std::vector<cv::Mat> &yf);

  cv::Mat ComplexMultiplication(const cv::Mat &x1, const cv::Mat &x2);

  cv::Mat ComplexDivision(const cv::Mat &x1, const cv::Mat &x2);

  // init only: Create Gaussian Peak as regression target
  cv::Mat CreateGaussianPeak(const int &sizey, const int &sizex);

  // init only: Discrete Fast Fourier Transform
  cv::Mat FFTD(cv::Mat img);

  // init only: get hann window
  cv::Mat CalculateHann(const cv::Size &sz);

  cv::Mat y_f_;
  cv::Mat cos_window_;

  const int kWindowSize_ = 50;
  const float kKernelSigma_ = 0.5f;
  const float kLambda_ = 0.0001f;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_
