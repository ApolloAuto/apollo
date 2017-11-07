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
#include <cstdio>
#include <assert.h>
#include <zlib.h>

namespace apollo {
namespace localization {
namespace msf {

const unsigned int ZlibStrategy::zlib_chunk = 16384;

unsigned int ZlibStrategy::encode(BufferStr& buf, BufferStr& buf_compressed) {
    return zlib_compress(buf, buf_compressed);
}

unsigned int ZlibStrategy::decode(BufferStr& buf, BufferStr& buf_uncompressed) {
    return zlib_uncompress(buf, buf_uncompressed);
}

unsigned int ZlibStrategy::zlib_compress(std::vector<unsigned char>& src,
                  std::vector<unsigned char>& dst) {
    dst.resize(zlib_chunk*2);
    int ret, flush;
    unsigned have;
    z_stream strm;
    unsigned char *in = &src[0];
    unsigned char *out = &dst[0];
    unsigned int src_idx = 0;
    unsigned int dst_idx = 0;

    /* allocate deflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = deflateInit(&strm, Z_BEST_SPEED); // Z_DEFAULT_COMPRESSION for better compression
    if (ret != Z_OK)
        return ret;

    /* compress until end of file */
    do {
        in = &src[src_idx];
        if (src.size() - src_idx > zlib_chunk) {
            strm.avail_in = zlib_chunk;
            flush = Z_NO_FLUSH;
        }
        else {
            strm.avail_in = src.size() - src_idx;
            flush = Z_FINISH;
        }
        strm.next_in = in;
        src_idx += strm.avail_in;

        /* run deflate() on input until output buffer not full, finish
           compression if all of source has been read in */
        do {
            strm.avail_out = zlib_chunk;
            strm.next_out = out;
            ret = deflate(&strm, flush);    /* no bad return value */
            assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
            have = zlib_chunk - strm.avail_out;
            dst_idx += have;
            if (dst_idx + zlib_chunk> dst.size()) {
                dst.resize(dst_idx + zlib_chunk*2);
            }
            out = &dst[dst_idx];
        } while (strm.avail_out == 0);
        assert(strm.avail_in == 0);     /* all input will be used */

        /* done when last data in file processed */
    } while (flush != Z_FINISH);
    assert(ret == Z_STREAM_END);        /* stream will be complete */

    /* clean up and return */
    (void)deflateEnd(&strm);
    dst.resize(dst_idx);
    return Z_OK;
}

unsigned int ZlibStrategy::zlib_uncompress(std::vector<unsigned char>& src,
                    std::vector<unsigned char>& dst) {
    dst.resize(zlib_chunk*2);
    int ret;
    unsigned have;
    z_stream strm;
    unsigned char *in = &src[0];
    unsigned char *out = &dst[0];
    unsigned int src_idx = 0;
    unsigned int dst_idx = 0;

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK)
        return ret;

    /* decompress until deflate stream ends or end of file */
    do {
        in = &src[src_idx];
        if (src.size() - src_idx > zlib_chunk) {
            strm.avail_in = zlib_chunk;
        }
        else {
            strm.avail_in = src.size() - src_idx;
        }
        strm.next_in = in;
        src_idx += strm.avail_in;
        if (strm.avail_in == 0)
            break;

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = zlib_chunk;
            strm.next_out = out;
            ret = inflate(&strm, Z_NO_FLUSH);
            assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;     /* and fall through */
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                return ret;
            }
            have = zlib_chunk - strm.avail_out;
            dst_idx += have;
            if (dst_idx + zlib_chunk > dst.size()) {
                dst.resize(dst_idx + zlib_chunk*2);
            }
            out = &dst[dst_idx];
        } while (strm.avail_out == 0);

        /* done when inflate() says it's done */
    } while (ret != Z_STREAM_END);

    /* clean up and return */
    (void)inflateEnd(&strm);
    dst.resize(dst_idx);
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

} // namespace msf
} // namespace localization
} // namespace apollo