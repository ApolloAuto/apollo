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

#pragma once

#include <vector>

namespace apollo {
namespace localization {
namespace msf {

class CompressionStrategy {
 public:
  typedef std::vector<unsigned char> BufferStr;
  virtual ~CompressionStrategy() {}
  virtual int Encode(BufferStr* buf, BufferStr* buf_compressed) = 0;
  virtual int Decode(BufferStr* buf, BufferStr* buf_uncompressed) = 0;

 protected:
};

class ZlibStrategy : public CompressionStrategy {
 public:
  virtual int Encode(BufferStr* buf, BufferStr* buf_compressed);
  virtual int Decode(BufferStr* buf, BufferStr* buf_uncompressed);

 protected:
  static const unsigned int zlib_chunk;
  int ZlibCompress(BufferStr* src, BufferStr* dst);
  int ZlibUncompress(BufferStr* src, BufferStr* dst);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
