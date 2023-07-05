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

#include "modules/perception/inference/tensorrt/batch_stream.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

BatchStream::BatchStream(int batchSize, int maxBatches, std::string dataPath)
    : mBatchSize(batchSize), mMaxBatches(maxBatches), mPath(dataPath) {
  FILE *file = fopen((mPath + "Batch0").c_str(), "rb");
  if (file != nullptr) {
    int d[4];
    int fs = static_cast<int>(fread(d, sizeof(int), 4, file));
    CHECK_EQ(fs, 4);
    mDims = nvinfer1::DimsNCHW{d[0], d[1], d[2], d[3]};
    fclose(file);
    mImageSize = mDims.c() * mDims.h() * mDims.w();
    mBatch.resize(mBatchSize * mImageSize, 0);
    mFileBatch.resize(mDims.n() * mImageSize, 0);
    reset(0);
  }
}

BatchStream::BatchStream() : mPath("") {}

void BatchStream::reset(int firstBatch) {
  if (mPath != "") {
    mBatchCount = 0;
    mFileCount = 0;
    mFileBatchPos = mDims.n();
    skip(firstBatch);
  }
}

bool BatchStream::next() {
  if (mBatchCount == mMaxBatches) {
    return false;
  }

  for (int csize = 1, batchPos = 0; batchPos < mBatchSize;
       batchPos += csize, mFileBatchPos += csize) {
    CHECK_GT(mFileBatchPos, 0);
    CHECK_LE(mFileBatchPos, mDims.n());
    // mMaxBatches > number of batches in the files
    if (mFileBatchPos == mDims.n() && !update()) {
      return false;
    }

    // copy the smaller of: elements left to fulfill the request,
    // or elements left in the file buffer.
    csize = std::min(mBatchSize - batchPos, mDims.n() - mFileBatchPos);
    std::copy_n(getFileBatch() + mFileBatchPos * mImageSize, csize * mImageSize,
                getBatch() + batchPos * mImageSize);
  }
  mBatchCount++;
  return true;
}

void BatchStream::skip(int skipCount) {
  if (mBatchSize >= mDims.n() && mBatchSize % mDims.n() == 0 &&
      mFileBatchPos == mDims.n()) {
    mFileCount += skipCount * mBatchSize / mDims.n();
    return;
  }

  int x = mBatchCount;
  for (int i = 0; i < skipCount; ++i) {
    next();
  }
  mBatchCount = x;
}

bool BatchStream::update() {
  std::string inputFileName = absl::StrCat(mPath, "Batch", mFileCount++);
  FILE *file = fopen(inputFileName.c_str(), "rb");
  if (file == nullptr) {
    return false;
  }

  int d[4];
  int fs = static_cast<int>(fread(d, sizeof(int), 4, file));
  CHECK_EQ(fs, 4);
  ACHECK(mDims.n() == d[0] && mDims.c() == d[1] && mDims.h() == d[2] &&
         mDims.w() == d[3]);

  size_t readInputCount =
      fread(getFileBatch(), sizeof(float), mDims.n() * mImageSize, file);
  CHECK_EQ(readInputCount, size_t(mDims.n() * mImageSize));
  fclose(file);
  mFileBatchPos = 0;
  return true;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
