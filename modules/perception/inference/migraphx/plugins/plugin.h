/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
 *
 *****************************************************************************/

#pragma once

#include <vector>

#define DEPRECATED

class Dims {
 public:
  static const int32_t MAX_DIMS = 8;
  int32_t nbDims;
  int32_t d[MAX_DIMS];
};

class Dims4 : public Dims {
 public:
  Dims4() {
    nbDims = 4;
    d[0] = d[1] = d[2] = d[3] = 0;
  }
  Dims4(int32_t d0, int32_t d1, int32_t d2, int32_t d3) {
    nbDims = 4;
    d[0] = d0;
    d[1] = d1;
    d[2] = d2;
    d[3] = d3;
  }
};

class IPlugin {
 public:
  virtual int32_t getNbOutputs() const = 0;
  virtual Dims getOutputDimensions(int32_t index, const Dims* inputs,
                                   int32_t nbInputDims) = 0;
  virtual void configure(const Dims* inputDims, int32_t nbInputs,
                         const Dims* outputDims, int32_t nbOutputs,
                         int32_t maxBatchSize) = 0;
  virtual int32_t initialize() = 0;
  virtual void terminate() = 0;
  virtual size_t getWorkspaceSize(int32_t maxBatchSize) const = 0;
  virtual int32_t enqueue(int32_t batchSize, const void* const* inputs,
                          void** outputs, void* workspace,
                          cudaStream_t stream) = 0;
  virtual size_t getSerializationSize() = 0;
  virtual void serialize(void* buffer) = 0;
  virtual ~IPlugin() {}
};
