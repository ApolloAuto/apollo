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

#pragma once

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
  BaseMapMatrix(const BaseMapMatrix& cell);
  /**@brief Initialize the map matrix. */
  virtual void Init(const BaseMapConfig* config) = 0;
  /**@brief Reset map cells data. */
  virtual void Reset(const BaseMapConfig* config) = 0;
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual unsigned int LoadBinary(unsigned char* buf) = 0;
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int CreateBinary(unsigned char* buf,
                                    unsigned int buf_size) const = 0;
  /**@brief Get the binary size of the object. */
  virtual unsigned int GetBinarySize() const = 0;
  /**@brief get intensity image of node. */
  virtual void GetIntensityImg(cv::Mat* intensity_img) const = 0;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
