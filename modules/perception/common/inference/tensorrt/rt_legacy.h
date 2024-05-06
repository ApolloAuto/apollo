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

#include <NvInferLegacyDims.h>

namespace nvinfer1 {
class DimsNCHW : public Dims4 {
 public:
    DimsNCHW() : Dims4() {}
    DimsNCHW(
      int32_t batch_size, int32_t channels,
      int32_t height, int32_t width)
        : Dims4(batch_size, channels, height, width) {}

    int32_t& n() {
      return d[0];
    }

    int32_t n() const {
      return d[0];
    }

    int32_t& c() {
      return d[1];
    }

    int32_t c() const {
      return d[1];
    }

    int32_t& h() {
      return d[2];
    }

    int32_t h() const {
      return d[2];
    }

    int32_t& w() {
      return d[3];
    }

    int32_t w() const {
      return d[3];
    }
};

class DimsCHW : public Dims3 {
 public:
    DimsCHW() : Dims3() {}
    DimsCHW(int32_t channels, int32_t height, int32_t width)
      : Dims3(channels, height, width) {}

    int32_t& c() {
      return d[0];
    }

    int32_t c() const {
      return d[0];
    }

    int32_t& h() {
      return d[1];
    }

    int32_t h() const {
      return d[1];
    }

    int32_t& w() {
      return d[2];
    }

    int32_t w() const {
      return d[2];
    }
};
}  // namespace nvinfer1

