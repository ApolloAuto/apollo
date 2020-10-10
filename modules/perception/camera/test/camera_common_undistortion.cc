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
#include "modules/perception/camera/test/camera_common_undistortion.h"

#include <npp.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

namespace apollo {
namespace perception {
namespace camera {

/* Initialization of the GPU routines for camera data preprocessing
 *
 * Return: 0 - success; other - failure
 * Params:  intrinsics_path - intrinsics path
 *          dev - dev number of the GPU device
 * Note: returns OK if already been inited.
 */
int ImageGpuPreprocessHandler::init(const std::string &intrinsics_path,
                                    int dev) {
  if (_inited) {
    return 0;
  }

  std::vector<double> D;
  std::vector<double> K;
  if (load_camera_intrinsics(intrinsics_path, &_width, &_height, &D, &K) != 0) {
    std::cerr << "Failed to load camera intrinsics!" << std::endl;
    return -1;
  }

  _dev_no = dev;

  if (set_device(_dev_no) != 0) {
    return -1;
  }

  size_t img_size = _height * _width;
  _in_size = static_cast<int>(img_size * CHANNEL * sizeof(uint8_t));

  _out_size = static_cast<int>(img_size * CHANNEL * sizeof(uint8_t));
  // if input is rgb, its size is same as output's
  BASE_CUDA_CHECK(cudaMalloc(&_d_rgb, _out_size));
  BASE_CUDA_CHECK(cudaMalloc(&_d_dst, _out_size));

  cv::Mat_<double> I = cv::Mat_<double>::eye(3, 3);
  cv::Mat map1(_height, _width, CV_32FC1);
  cv::Mat map2(_height, _width, CV_32FC1);

  cv::Mat cm = cv::Mat(3, 3, CV_64F, K.data()).clone();
  cv::Mat dc = cv::Mat(1, 5, CV_64F, D.data()).clone();
  cv::Mat ncm = cm.clone();

  cv::initUndistortRectifyMap(cm, dc, I, ncm, cv::Size(_width, _height),
                              CV_32FC1, map1, map2);

  std::vector<float> mapx;
  std::vector<float> mapy;
  mapx.resize(img_size);
  mapy.resize(img_size);

  for (int r = 0; r < _height; ++r) {
    for (int c = 0; c < _width; ++c) {
      mapx[r * _width + c] = map1.at<float>(r, c);
      mapy[r * _width + c] = map2.at<float>(r, c);
    }
  }

  BASE_CUDA_CHECK(cudaMalloc(&_d_mapx, img_size * sizeof(float)));
  BASE_CUDA_CHECK(cudaMalloc(&_d_mapy, img_size * sizeof(float)));

  BASE_CUDA_CHECK(cudaMemcpy(_d_mapx, mapx.data(), img_size * sizeof(float),
                             cudaMemcpyHostToDevice));
  BASE_CUDA_CHECK(cudaMemcpy(_d_mapy, mapy.data(), img_size * sizeof(float),
                             cudaMemcpyHostToDevice));
  _inited = true;
  return 0;
}

/* Processing each image
 *
 * Return: 0 - success; other - failure
 * Params: src - input image array (of type yuyv)
 *         dst - output image array (of type rgb)
 *
 */
int ImageGpuPreprocessHandler::handle(uint8_t *src, uint8_t *dst) {
  if (!_inited) {
    return -1;
  }

  BASE_CUDA_CHECK(cudaMemcpy(_d_rgb, src, _in_size, cudaMemcpyHostToDevice));

  NppiSize Remapsize;
  NppiInterpolationMode RemapMode = NPPI_INTER_LINEAR;
  Remapsize.width = _width;
  Remapsize.height = _height;
  NppiRect RemapRect = {0, 0, _width, _height};

  NppStatus eStatusNPP =
      nppiRemap_8u_C3R(_d_rgb, Remapsize, (_width * CHANNEL), RemapRect,
                       _d_mapx, (_width * static_cast<int>(sizeof(float))),
                       _d_mapy, (_width * static_cast<int>(sizeof(float))),
                       _d_dst, (_width * CHANNEL), Remapsize, RemapMode);

  if (eStatusNPP != NPP_SUCCESS) {
    std::cerr << "NPP_CHECK_NPP - eStatusNPP = " << eStatusNPP << std::endl;
    return static_cast<int>(eStatusNPP);
  }

  BASE_CUDA_CHECK(cudaMemcpy(dst, _d_dst, _out_size, cudaMemcpyDeviceToHost));
  return 0;
}

/* Release the resources
 *
 * Return: 0 - success; other - failure
 *
 */
int ImageGpuPreprocessHandler::release(void) {
  if (_d_mapy) {
    BASE_CUDA_CHECK(cudaFree(_d_mapy));
    _d_mapy = nullptr;
  }
  if (_d_mapx) {
    BASE_CUDA_CHECK(cudaFree(_d_mapx));
    _d_mapx = nullptr;
  }
  if (_d_dst) {
    BASE_CUDA_CHECK(cudaFree(_d_dst));
    _d_dst = nullptr;
  }
  if (_d_rgb) {
    BASE_CUDA_CHECK(cudaFree(_d_rgb));
    _d_rgb = nullptr;
  }
  _inited = false;
  return 0;
}

int ImageGpuPreprocessHandler::load_camera_intrinsics(
    const std::string &intrinsics_path, int *width, int *height,
    std::vector<double> *D, std::vector<double> *K) {
  if (!(boost::filesystem::exists(intrinsics_path))) {
    return -1;
  }
  YAML::Node node = YAML::LoadFile(intrinsics_path);
  if (node.IsNull()) {
    return -1;
  }
  D->resize(node["D"].size());
  for (int i = 0; i < static_cast<int>(node["D"].size()); ++i) {
    (*D)[i] = node["D"][i].as<double>();
  }
  K->resize(node["K"].size());
  for (int i = 0; i < static_cast<int>(node["K"].size()); ++i) {
    (*K)[i] = node["K"][i].as<double>();
  }
  *width = node["width"].as<int>();
  *height = node["height"].as<int>();
  return 0;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
