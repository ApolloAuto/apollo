/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/common/util/compression.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

TEST(CompressionTestSuite, ZlibStrategyTest) {
  ZlibStrategy zlib;
  std::vector<unsigned char> buf_uncompressed;
  std::vector<unsigned char> buf_compressed;
  for (int i = 0; i < 255; i++) {
    buf_uncompressed.push_back((unsigned char)i);
  }

  std::vector<unsigned char> buf_uncompressed2;
  zlib.Encode(&buf_uncompressed, &buf_compressed);
  zlib.Decode(&buf_compressed, &buf_uncompressed2);

  for (int i = 0; i < 255; i++) {
    EXPECT_EQ(buf_uncompressed2[i], (unsigned char)i);
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
