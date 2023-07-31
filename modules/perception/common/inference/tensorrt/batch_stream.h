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

#include <NvInferVersion.h>

#ifdef NV_TENSORRT_MAJOR
    #if NV_TENSORRT_MAJOR == 8
    #include "modules/perception/common/inference/tensorrt/rt_legacy.h"
    #endif
#endif

#include <string>
#include <vector>

#include "NvInfer.h"

namespace apollo {
namespace perception {
namespace inference {

class BatchStream {
 public:
  BatchStream(int batchSize, int maxBatches, std::string data_path);
  BatchStream();

  // @brief reset current batch id
  void reset(int firstBatch);

  // @brief get next batch
  bool next();

  // @brief skip `skipCount` batches
  void skip(int skipCount);

  float *getBatch() { return &mBatch[0]; }
  int getBatchesRead() const { return mBatchCount; }
  int getBatchSize() const { return mBatchSize; }
  nvinfer1::DimsNCHW getDims() const { return mDims; }

 private:
  float *getFileBatch() { return &mFileBatch[0]; }

  bool update();

  int mBatchSize{0};
  int mMaxBatches{0};
  int mBatchCount{0};

  int mFileCount{0};
  int mFileBatchPos{0};
  int mImageSize{0};

  nvinfer1::DimsNCHW mDims;
  std::vector<float> mBatch;
  std::vector<float> mFileBatch;
  std::string mPath;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
