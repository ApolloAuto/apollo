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

 //
// Basic components for running KCF tracker (get feature, detect, train)
//
// Reference
//
// 1. Henriques, João F., et al. "High-speed tracking with kernelized correlation filters."
// IEEE Transactions on Pattern Analysis and Machine Intelligence 37.3 (2015): 583-596.
//
// 2. https://github.com/foolwood/KCF
//
// 3. https://github.com/joaofaro/KCFcpp
//

#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_COMPONENTS_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_COMPONENTS_H

#include <cmath>

#include <opencv2/opencv.hpp>

#include "obstacle/camera/tracker/mix_camera_tracker/base_affinity_tracker.h"
//#include "obstacle/camera/tracker/mix_camera_tracker/kcf/kcf_fhog.h"

namespace adu {
namespace perception {
namespace obstacle {

class KCFComponents {
public:

    KCFComponents() {}

    bool init();

    // Get x_f or z_f
    bool get_feature(const cv::Mat &img, const cv::Rect &box, std::vector<cv::Mat> &feature);

    // Get response score
    bool detect(const Tracked &tracked_obj, const std::vector<cv::Mat> &z_f, float &score);

    // Get alpha_f
    bool train(const cv::Mat &img, Tracked &tracked_obj);

private:

    cv::Mat gaussian_correlation(std::vector<cv::Mat> xf, std::vector<cv::Mat> yf);

    cv::Mat complex_multiplication(const cv::Mat &x1, const cv::Mat &x2);

    cv::Mat complex_division(const cv::Mat &x1, const cv::Mat &x2);

    // init only: Create Gaussian Peak as regression target
    cv::Mat create_gaussian_peak(int sizey, int sizex);

    // init only: Discrete Fast Fourier Transform
    cv::Mat fftd(cv::Mat img);

    // init only: get hann window
    cv::Mat calculate_hann(const cv::Size &sz);

    cv::Mat _y_f;

    cv::Mat _cos_window;

    // TODO  Put patch size into config and init
    int _window_size = 50; // 100, 50

    int _cell_size = 4;

    float _kernel_sigma = 0.5f;

    float _lambda = 0.0001f;

//	HogFeature _hog_extractor;
};

} //namespace adu
} //namespace perception
} //namespace obstacle

#endif //ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_COMPONENTS_H
