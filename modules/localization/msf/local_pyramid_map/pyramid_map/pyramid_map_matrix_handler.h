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

#include <memory>

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix_handler.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapMatrixHandlerSelector {
 public:
  PyramidMapMatrixHandlerSelector();
  ~PyramidMapMatrixHandlerSelector();

  static BaseMapMatrixHandler* AllocPyramidMapMatrixHandler(MapVersion version);
};

class LossyMapMatrixHandler : public BaseMapMatrixHandler {
 public:
  LossyMapMatrixHandler();
  virtual ~LossyMapMatrixHandler();

 protected:
  virtual unsigned char EncodeIntensity(float intensity) const;
  virtual void DecodeIntensity(unsigned char data, float* intensity) const;
  virtual uint16_t EncodeIntensityVar(float var) const;
  virtual void DecodeIntensityVar(uint16_t data, float* var) const;
  virtual uint16_t EncodeAltitude(float altitude, float min_altitude,
                                  float altitude_interval) const;
  virtual void DecodeAltitude(uint16_t data, float min_altitude,
                              float altitude_interval, float* altitude) const;
  virtual unsigned char EncodeCount(unsigned int count,
                                    unsigned int count_range) const;
  virtual void DecodeCount(unsigned char data, unsigned int* count) const;
  const unsigned int var_range_ = 1023;  // 65535;
  const unsigned int var_ratio_ = 4;     // 256;
  const unsigned int count_range_ = 2;   // 31;
  const float ground_alt_interval_ = 0.04f;
  const float alt_avg_interval_ = 0.04f;
  mutable float alt_avg_min_ = 0.0f;
  mutable float ground_alt_min_ = 0.0f;
  mutable float alt_avg_max_ = 0.0f;
  mutable float ground_alt_max_ = 0.0f;
};

class LossyMapFullAltMatrixHandler : public LossyMapMatrixHandler {
 public:
  LossyMapFullAltMatrixHandler();
  ~LossyMapFullAltMatrixHandler();

  virtual size_t LoadBinary(const unsigned char* buf,
                            std::shared_ptr<BaseMapMatrix> matrix);
  virtual size_t CreateBinary(const std::shared_ptr<BaseMapMatrix> matrix,
                              unsigned char* buf, size_t buf_size);
  virtual size_t GetBinarySize(const std::shared_ptr<BaseMapMatrix> matrix);
};

class LosslessMapMatrixHandler : public BaseMapMatrixHandler {
 public:
  LosslessMapMatrixHandler();
  ~LosslessMapMatrixHandler();
  virtual size_t LoadBinary(const unsigned char* buf,
                            std::shared_ptr<BaseMapMatrix> matrix);
  virtual size_t CreateBinary(const std::shared_ptr<BaseMapMatrix> matrix,
                              unsigned char* buf, size_t buf_size);
  virtual size_t GetBinarySize(const std::shared_ptr<BaseMapMatrix> matrix);
};

class PyramidLossyMapMatrixHandler : public LossyMapMatrixHandler {
 public:
  PyramidLossyMapMatrixHandler();
  ~PyramidLossyMapMatrixHandler();
  virtual size_t LoadBinary(const unsigned char* buf,
                            std::shared_ptr<BaseMapMatrix> matrix);
  virtual size_t CreateBinary(const std::shared_ptr<BaseMapMatrix> matrix,
                              unsigned char* buf, size_t buf_size);
  virtual size_t GetBinarySize(const std::shared_ptr<BaseMapMatrix> matrix);
};

class PyramidLosslessMapMatrixHandler : public BaseMapMatrixHandler {
 public:
  PyramidLosslessMapMatrixHandler();
  ~PyramidLosslessMapMatrixHandler();
  virtual size_t LoadBinary(const unsigned char* buf,
                            std::shared_ptr<BaseMapMatrix> matrix);
  virtual size_t CreateBinary(const std::shared_ptr<BaseMapMatrix> matrix,
                              unsigned char* buf, size_t buf_size);
  virtual size_t GetBinarySize(const std::shared_ptr<BaseMapMatrix> matrix);
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
