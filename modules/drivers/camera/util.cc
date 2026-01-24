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

#include "modules/drivers/camera/util.h"

#include <cstdarg>
#include <cstdint>

namespace apollo {
namespace drivers {
namespace camera {

void print_m256(__m256i a) {
    unsigned char snoop[32];
    bool dst_align = Aligned(reinterpret_cast<void*>(snoop));
    if (dst_align) {
        Store<true>(reinterpret_cast<__m256i*>(snoop), a);
    }
    else {
        Store<false>(reinterpret_cast<__m256i*>(snoop), a);
    }
    for (int i = 0; i < 32; ++i) {
        printf("DEBUG8 %d %u \n", i, snoop[i]);
    }
}
void print_m256_i32(const __m256i a) {
    unsigned int snoop[8];
    bool dst_align = Aligned(reinterpret_cast<void*>(snoop));
    if (dst_align)
        Store<true>(reinterpret_cast<__m256i*>(snoop), a);
    else
        Store<false>(reinterpret_cast<__m256i*>(snoop), a);
    for (int i = 0; i < 8; ++i) {
        printf("DEBUG32 %d %u \n", i, snoop[i]);
    }
}

void print_m256_i16(const __m256i a) {
    uint16_t snoop[16];
    bool dst_align = Aligned(reinterpret_cast<void*>(snoop));
    if (dst_align)
        Store<true>(reinterpret_cast<__m256i*>(snoop), a);
    else
        Store<false>(reinterpret_cast<__m256i*>(snoop), a);
    for (int i = 0; i < 16; ++i) {
        printf("DEBUG16 %d %u \n", i, snoop[i]);
    }
}

template <bool align>
SIMD_INLINE void yuv_separate_avx2(uint8_t* y, __m256i* y0, __m256i* y1, __m256i* u0, __m256i* v0) {
    __m256i yuv_m256[4];

    if (align) {
        yuv_m256[0] = Load<true>(reinterpret_cast<__m256i*>(y));
        yuv_m256[1] = Load<true>(reinterpret_cast<__m256i*>(y) + 1);
        yuv_m256[2] = Load<true>(reinterpret_cast<__m256i*>(y) + 2);
        yuv_m256[3] = Load<true>(reinterpret_cast<__m256i*>(y) + 3);
    } else {
        yuv_m256[0] = Load<false>(reinterpret_cast<__m256i*>(y));
        yuv_m256[1] = Load<false>(reinterpret_cast<__m256i*>(y) + 1);
        yuv_m256[2] = Load<false>(reinterpret_cast<__m256i*>(y) + 2);
        yuv_m256[3] = Load<false>(reinterpret_cast<__m256i*>(y) + 3);
    }

    *y0 = _mm256_or_si256(
            _mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[0], Y_SHUFFLE0), 0xD8),
            _mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[1], Y_SHUFFLE1), 0xD8));
    *y1 = _mm256_or_si256(
            _mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[2], Y_SHUFFLE0), 0xD8),
            _mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[3], Y_SHUFFLE1), 0xD8));

    *u0 = _mm256_permutevar8x32_epi32(
            _mm256_or_si256(
                    _mm256_or_si256(
                            _mm256_shuffle_epi8(yuv_m256[0], U_SHUFFLE0), _mm256_shuffle_epi8(yuv_m256[1], U_SHUFFLE1)),
                    _mm256_or_si256(
                            _mm256_shuffle_epi8(yuv_m256[2], U_SHUFFLE2),
                            _mm256_shuffle_epi8(yuv_m256[3], U_SHUFFLE3))),
            U_SHUFFLE4);
    *v0 = _mm256_permutevar8x32_epi32(
            _mm256_or_si256(
                    _mm256_or_si256(
                            _mm256_shuffle_epi8(yuv_m256[0], V_SHUFFLE0), _mm256_shuffle_epi8(yuv_m256[1], V_SHUFFLE1)),
                    _mm256_or_si256(
                            _mm256_shuffle_epi8(yuv_m256[2], V_SHUFFLE2),
                            _mm256_shuffle_epi8(yuv_m256[3], V_SHUFFLE3))),
            U_SHUFFLE4);
}

template <bool align>
void yuv2rgb_avx2(__m256i y0, __m256i u0, __m256i v0, uint8_t* rgb) {
    __m256i r0 = YuvToRed(y0, v0);
    __m256i g0 = YuvToGreen(y0, u0, v0);
    __m256i b0 = YuvToBlue(y0, u0);

    Store<align>(reinterpret_cast<__m256i*>(rgb) + 0, InterleaveBgr<0>(r0, g0, b0));
    Store<align>(reinterpret_cast<__m256i*>(rgb) + 1, InterleaveBgr<1>(r0, g0, b0));
    Store<align>(reinterpret_cast<__m256i*>(rgb) + 2, InterleaveBgr<2>(r0, g0, b0));
}

template <bool align>
void yuv2rgb_avx2(uint8_t* yuv, uint8_t* rgb) {
    __m256i y0, y1, u0, v0;

    yuv_separate_avx2<align>(yuv, &y0, &y1, &u0, &v0);
    __m256i u0_u0 = _mm256_permute4x64_epi64(u0, 0xD8);
    __m256i v0_v0 = _mm256_permute4x64_epi64(v0, 0xD8);
    yuv2rgb_avx2<align>(y0, _mm256_unpacklo_epi8(u0_u0, u0_u0), _mm256_unpacklo_epi8(v0_v0, v0_v0), rgb);
    yuv2rgb_avx2<align>(
            y1, _mm256_unpackhi_epi8(u0_u0, u0_u0), _mm256_unpackhi_epi8(v0_v0, v0_v0), rgb + 3 * sizeof(__m256i));
}

void yuyv2rgb_avx(unsigned char* YUV, unsigned char* RGB, int NumPixels) {
    assert(NumPixels == (1920 * 1080));
    bool align = Aligned(YUV) & Aligned(RGB);
    uint8_t* yuv_offset = YUV;
    uint8_t* rgb_offset = RGB;
    if (align) {
        for (int i = 0; i < NumPixels; i = i + static_cast<int>(2 * sizeof(__m256i)),
                 yuv_offset += 4 * static_cast<int>(sizeof(__m256i)),
                 rgb_offset += 6 * static_cast<int>(sizeof(__m256i))) {
            yuv2rgb_avx2<true>(yuv_offset, rgb_offset);
        }
    } else {
        for (int i = 0; i < NumPixels; i = i + static_cast<int>(2 * sizeof(__m256i)),
                 yuv_offset += 4 * static_cast<int>(sizeof(__m256i)),
                 rgb_offset += 6 * static_cast<int>(sizeof(__m256i))) {
            yuv2rgb_avx2<false>(yuv_offset, rgb_offset);
        }
    }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
