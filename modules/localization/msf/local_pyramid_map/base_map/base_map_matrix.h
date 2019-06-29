/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <assert.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of the map cells in a map node. */
class BaseMapMatrix {
 public:
  /**@brief The default constructor. */
  BaseMapMatrix();
  /**@brief The deconstructor. */
  virtual ~BaseMapMatrix();
  /**@brief The copy constructor. */
  explicit BaseMapMatrix(const BaseMapMatrix& map_matrix);
  /**@brief Initialize the map matrix. */
  virtual void Init(const BaseMapConfig& config) = 0;
  /**@brief Reset map cells data. */
  virtual void Reset() = 0;
  /**@brief get intensity image of node. */
  virtual bool GetIntensityImg(cv::Mat* intensity_img) const;
  /**@brief get altitude image of node. */
  virtual bool GetAltitudeImg(cv::Mat* altitude_img) const;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
