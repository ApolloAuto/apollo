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
//

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_

#include <opencv2/opencv.hpp>
#include <cmath>

#include "obstacle/camera/tracker/mix_camera_tracker/base_affinity_tracker.h"

namespace apollo {
namespace perception {

class KCFComponents {
 public:
  KCFComponents() {}

  bool Init();

  // Get x_f or z_f
  bool GetFeatures(const cv::Mat &img, const cv::Rect &box,
                   std::vector<cv::Mat>* feature);

  // Get response score
  bool Detect(const Tracked &tracked_obj, const std::vector<cv::Mat> &z_f,
              float* score);

  // Get alpha_f
  bool Train(const cv::Mat &img, Tracked* tracked_obj);

 private:
  cv::Mat GaussianCorrelation(const std::vector<cv::Mat> &xf,
                              const std::vector<cv::Mat> &yf);

  cv::Mat ComplexMultiplication(const cv::Mat &x1, const cv::Mat &x2);

  cv::Mat ComplexDivision(const cv::Mat &x1, const cv::Mat &x2);

  // init only: Create Gaussian Peak as regression target
  cv::Mat CreateGaussianPeak(int sizey, int sizex);

  // init only: Discrete Fast Fourier Transform
  cv::Mat FFTD(cv::Mat img);

  // init only: get hann window
  cv::Mat CalculateHann(const cv::Size &sz);

  cv::Mat y_f_;
  cv::Mat cos_window_;

  int window_size_ = 50;
  int cell_size_ = 4;
  float kernel_sigma_ = 0.5f;
  float lambda_ = 0.0001f;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRACKER_KCF_COMPONENTS_H_
