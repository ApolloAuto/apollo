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

#include <zlib.h>

#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

const unsigned int ZlibStrategy::zlib_chunk = 16384;

int ZlibStrategy::Encode(BufferStr* buf, BufferStr* buf_compressed) {
  return ZlibCompress(buf, buf_compressed);
}

int ZlibStrategy::Decode(BufferStr* buf, BufferStr* buf_uncompressed) {
  return ZlibUncompress(buf, buf_uncompressed);
}

int ZlibStrategy::ZlibCompress(BufferStr* src, BufferStr* dst) {
  dst->resize(zlib_chunk * 2);
  int ret, flush;
  unsigned have;
  z_stream stream_data;
  unsigned char* in = &((*src)[0]);
  unsigned char* out = &((*dst)[0]);
  unsigned int src_idx = 0;
  unsigned int dst_idx = 0;

  /* allocate deflate state */
  stream_data.zalloc = Z_NULL;
  stream_data.zfree = Z_NULL;
  stream_data.opaque = Z_NULL;
  ret = deflateInit(&stream_data, Z_BEST_SPEED);
  if (ret != Z_OK) {
    return ret;
  }
  /* compress until end of file */
  do {
    in = &((*src)[src_idx]);
    if (src->size() - src_idx > zlib_chunk) {
      stream_data.avail_in = zlib_chunk;
      flush = Z_NO_FLUSH;
    } else {
      stream_data.avail_in = static_cast<unsigned int>(src->size()) - src_idx;
      flush = Z_FINISH;
    }
    stream_data.next_in = in;
    src_idx += stream_data.avail_in;

    /* run deflate() on input until output buffer not full, finish
       compression if all of source has been read in */
    do {
      stream_data.avail_out = zlib_chunk;
      stream_data.next_out = out;
      ret = deflate(&stream_data, flush); /* no bad return value */
      DCHECK_NE(ret, Z_STREAM_ERROR);     /* state not clobbered */
      have = zlib_chunk - stream_data.avail_out;
      dst_idx += have;
      if (dst_idx + zlib_chunk > dst->size()) {
        dst->resize(dst_idx + zlib_chunk * 2);
      }
      out = &((*dst)[dst_idx]);
    } while (stream_data.avail_out == 0);
    DCHECK_EQ(stream_data.avail_in, 0); /* all input will be used */

    /* done when last data in file processed */
  } while (flush != Z_FINISH);
  DCHECK_EQ(ret, Z_STREAM_END); /* stream will be complete */

  /* clean up and return */
  (void)deflateEnd(&stream_data);
  dst->resize(dst_idx);
  return Z_OK;
}

int ZlibStrategy::ZlibUncompress(BufferStr* src, BufferStr* dst) {
  dst->resize(zlib_chunk * 2);
  int ret;
  unsigned have;
  z_stream stream_data;
  unsigned char* in = &((*src)[0]);
  unsigned char* out = &((*dst)[0]);
  unsigned int src_idx = 0;
  unsigned int dst_idx = 0;

  /* allocate inflate state */
  stream_data.zalloc = Z_NULL;
  stream_data.zfree = Z_NULL;
  stream_data.opaque = Z_NULL;
  stream_data.avail_in = 0;
  stream_data.next_in = Z_NULL;
  ret = inflateInit(&stream_data);
  if (ret != Z_OK) {
    return ret;
  }
  /* decompress until deflate stream ends or end of file */
  do {
    in = &((*src)[src_idx]);
    if (src->size() - src_idx > zlib_chunk) {
      stream_data.avail_in = zlib_chunk;
    } else {
      stream_data.avail_in = static_cast<unsigned int>(src->size()) - src_idx;
    }
    stream_data.next_in = in;
    src_idx += stream_data.avail_in;
    if (stream_data.avail_in == 0) {
      break;
    }
    /* run inflate() on input until output buffer not full */
    do {
      stream_data.avail_out = zlib_chunk;
      stream_data.next_out = out;
      ret = inflate(&stream_data, Z_NO_FLUSH);
      DCHECK_NE(ret, Z_STREAM_ERROR); /* state not clobbered */
      switch (ret) {
        case Z_NEED_DICT:
          ret = Z_DATA_ERROR; /* and fall through */
        case Z_DATA_ERROR:
        case Z_MEM_ERROR:
          (void)inflateEnd(&stream_data);
          return ret;
      }
      have = zlib_chunk - stream_data.avail_out;
      dst_idx += have;
      if (dst_idx + zlib_chunk > dst->size()) {
        dst->resize(dst_idx + zlib_chunk * 2);
      }
      out = &((*dst)[dst_idx]);
    } while (stream_data.avail_out == 0);

    /* done when inflate() says it's done */
  } while (ret != Z_STREAM_END);

  /* clean up and return */
  (void)inflateEnd(&stream_data);
  dst->resize(dst_idx);
  return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
