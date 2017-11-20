/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "usb_cam/utility.h"
#include <stdarg.h>

namespace usb_cam {

#define YUV_INT32

void errno_exit(const char * s)
{
  ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}


/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  val = val < 0 ? 0 : val;
  return val > 255 ? 255 : val;

  // New method (array)
  //return uchar_clipping_table[val + clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char* r,
                    unsigned char* g, unsigned char* b)
{
  #ifdef YUV_INT32
	const int y2 = (int)y;
	const int u2 = (int)u - 128;
	const int v2 = (int)v - 128;
	
	
	int r2 = (y2 * (Y_TO_RGB_WEIGHT) + (V_TO_RED_WEIGHT * v2)) >> YUV_TO_BGR_AVERAGING_SHIFT;
	int g2 = (y2 * (Y_TO_RGB_WEIGHT) + ((U_TO_GREEN_WEIGHT * u2) + (V_TO_GREEN_WEIGHT * v2))) >> YUV_TO_BGR_AVERAGING_SHIFT;
	int b2 = (y2 * (Y_TO_RGB_WEIGHT) + ((U_TO_BLUE_WEIGHT * u2))) >> YUV_TO_BGR_AVERAGING_SHIFT;
	*r = CLIPVALUE(r2);
	*g = CLIPVALUE(g2);
	*b = CLIPVALUE(b2);

#else
	const int y2 = (int)y;
	const int u2 = (int)u - 128;
	const int v2 = (int)v - 128;

	double r2 = y2 + (1.4065 * v2);
	double g2 = y2 - (0.3455 * u2) - (0.7169 * v2);
	double b2 = y2 + (2.041  * u2);


	*r = CLIPVALUE(r2);
	*g = CLIPVALUE(g2);
	*b = CLIPVALUE(b2);
#endif

}

void uyvy2rgb(char *YUV, char *RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
  {
    u = (unsigned char)YUV[i + 0];
    y0 = (unsigned char)YUV[i + 1];
    v = (unsigned char)YUV[i + 2];
    y1 = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

void mono102mono8(char *RAW, char *MONO, int NumPixels)
{
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1)
  {
    //first byte is low byte, second byte is high byte; smash together and convert to 8-bit
    MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
  }
}

void yuyv2rgb(char *YUV, char *RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
  {
    y0 = (unsigned char)YUV[i + 0];
    u = (unsigned char)YUV[i + 1];
    y1 = (unsigned char)YUV[i + 2];
    v = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

void yuyv2rgb_(unsigned char *YUV, unsigned char *RGB, int i_startIndx, int j_startIndx, int i_endIndx)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = i_startIndx, j = j_startIndx; i < i_endIndx; i += 4, j += 6)
  {
    y0 = (unsigned char)YUV[i + 0];
    u = (unsigned char)YUV[i + 1];
    y1 = (unsigned char)YUV[i + 2];
    v = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
  
}

void yuyv2rgb_thread(unsigned char *YUV, unsigned char *RGB, int NumPixels)
{
    // ToDO: hack added by Bocongl
    int segment = NumPixels * 2 / 3;
    boost::thread worker1(boost::bind(&yuyv2rgb_, YUV, RGB, 
                0, 0, segment));
    boost::thread worker2(boost::bind(&yuyv2rgb_, YUV, RGB, 
                segment, (segment / 4) * 6, segment * 2));
    boost::thread worker3(boost::bind(&yuyv2rgb_, YUV, RGB, 
                segment * 2, (segment / 2) * 6, segment * 3));
    worker1.join();
    worker2.join();
    worker3.join();
    /*
    int segment = NumPixels;
    boost::thread worker1(boost::bind(&yuyv2rgb_, YUV, RGB, 0, 0, segment));
    boost::thread worker2(boost::bind(&yuyv2rgb_, YUV, RGB, segment, (segment * 3) / 2, segment * 2));
    worker1.join();
    worker2.join();
    */
}

void yuyv2rgb_nothread(unsigned char *YUV, unsigned char *RGB, int NumPixels)
{
    yuyv2rgb_(YUV, RGB, 0, 0, NumPixels*2);
}

void rgb242rgb(char *YUV, char *RGB, int NumPixels)
{
  memcpy(RGB, YUV, NumPixels * 3);
}

void print_m256( __m256i a)
{
	unsigned char snoop[32];
	bool dst_align = Aligned((void*)snoop);
	if (dst_align)
		Store<true>((__m256i*)snoop, a);
	else
		Store<false>((__m256i*)snoop, a);
	for (int i = 0; i < 32; i++)
	{
		printf("DEBUG8 %d %d \n", i, snoop[i]);
	}
}
void print_m256_i32(const __m256i a)
{
	unsigned int snoop[8];
	bool dst_align = Aligned((void*)snoop);
	if (dst_align)
		Store<true>((__m256i*)snoop, a);
	else
		Store<false>((__m256i*)snoop, a);
	for (int i = 0; i < 8; i++)
	{
		printf("DEBUG32 %d %d \n", i, snoop[i]);
	}
}

void print_m256_i16(const __m256i a)
{
	unsigned short snoop[16];
	bool dst_align = Aligned((void*)snoop);
	if (dst_align)
		Store<true>((__m256i*)snoop, a);
	else
		Store<false>((__m256i*)snoop, a);
	for (int i = 0; i < 16; i++)
	{
		printf("DEBUG16 %d %d \n", i, snoop[i]);
	}
}

template <bool align> SIMD_INLINE void yuv_seperate_avx2(uint8_t* y,__m256i & y0 , __m256i & y1 , __m256i & u0, __m256i & v0)
{
	__m256i yuv_m256[4];
	
	if (align)
	{
		yuv_m256[0] = Load<true>((__m256i*)y);
		yuv_m256[1] = Load<true>((__m256i*)y + 1);
		yuv_m256[2] = Load<true>((__m256i*)y + 2);
		yuv_m256[3] = Load<true>((__m256i*)y + 3);
	}
	else
	{
		yuv_m256[0] = Load<false>((__m256i*)y);
		yuv_m256[1] = Load<false>((__m256i*)y + 1);
		yuv_m256[2] = Load<false>((__m256i*)y + 2);
		yuv_m256[3] = Load<false>((__m256i*)y + 3);
	}

	y0 = _mm256_or_si256(_mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[0], Y_SHUFFLE0), 0xD8),
		_mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[1], Y_SHUFFLE1), 0xD8));
	y1 = _mm256_or_si256(_mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[2], Y_SHUFFLE0), 0xD8),
		_mm256_permute4x64_epi64(_mm256_shuffle_epi8(yuv_m256[3], Y_SHUFFLE1), 0xD8));

	u0 = _mm256_permutevar8x32_epi32(_mm256_or_si256(_mm256_or_si256(_mm256_shuffle_epi8(yuv_m256[0], U_SHUFFLE0),
		_mm256_shuffle_epi8(yuv_m256[1], U_SHUFFLE1)),
		_mm256_or_si256(_mm256_shuffle_epi8(yuv_m256[2], U_SHUFFLE2),
			_mm256_shuffle_epi8(yuv_m256[3], U_SHUFFLE3))), U_SHUFFLE4);
	v0 = _mm256_permutevar8x32_epi32(_mm256_or_si256(_mm256_or_si256(_mm256_shuffle_epi8(yuv_m256[0], V_SHUFFLE0),
		_mm256_shuffle_epi8(yuv_m256[1], V_SHUFFLE1)),
		_mm256_or_si256(_mm256_shuffle_epi8(yuv_m256[2], V_SHUFFLE2),
			_mm256_shuffle_epi8(yuv_m256[3], V_SHUFFLE3))), U_SHUFFLE4);
}

template <bool align> void yuv2rgb_avx2(__m256i y0, __m256i u0, __m256i v0, uint8_t * rgb)
{
	__m256i r0 = YuvToRed(y0, v0);
	__m256i g0 = YuvToGreen(y0, u0, v0);
	__m256i b0 = YuvToBlue(y0, u0);

	Store<align>((__m256i *)rgb + 0, InterleaveBgr<0>(r0, g0, b0));
	Store<align>((__m256i *)rgb + 1, InterleaveBgr<1>(r0, g0, b0));
	Store<align>((__m256i *)rgb + 2, InterleaveBgr<2>(r0, g0, b0));
}

template <bool align> void yuv2rgb_avx2(uint8_t* yuv,uint8_t* rgb)
{
	__m256i y0, y1, u0, v0;

	
	yuv_seperate_avx2<align>(yuv, y0, y1, u0, v0);
	__m256i u0_u0 = _mm256_permute4x64_epi64(u0, 0xD8);
	__m256i v0_v0 = _mm256_permute4x64_epi64(v0, 0xD8);
	yuv2rgb_avx2<align>(y0, _mm256_unpacklo_epi8(u0_u0, u0_u0), _mm256_unpacklo_epi8(v0_v0, v0_v0), rgb);
	yuv2rgb_avx2<align>(y1, _mm256_unpackhi_epi8(u0_u0, u0_u0), _mm256_unpackhi_epi8(v0_v0, v0_v0), rgb + 3 * sizeof(__m256i));
}

 void yuyv2rgb_avx(unsigned char *YUV, unsigned char *RGB, int NumPixels)
{
	 assert(NumPixels == (1920 * 1080));
	 bool align = Aligned(YUV) & Aligned(RGB);
	 uint8_t* yuv_offset = YUV;
	 uint8_t* rgb_offset = RGB;
	 if (align) 
	 {
		 for (int i = 0; i < NumPixels; i = i + (2 * sizeof(__m256i)), yuv_offset += 4 * sizeof(__m256i), rgb_offset += 6 * sizeof(__m256i))
		 {
			 yuv2rgb_avx2<true>(yuv_offset, rgb_offset);
		 }
	 }
	 else
	 {
		 for (int i = 0; i < NumPixels; i = i + (2 * sizeof(__m256i)), yuv_offset += 4 * sizeof(__m256i), rgb_offset += 6 * sizeof(__m256i))
		 {
			 yuv2rgb_avx2<false>(yuv_offset, rgb_offset);
		 }
	 }

}

}



