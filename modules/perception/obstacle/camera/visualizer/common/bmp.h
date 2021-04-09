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

#ifndef MODULES_PERCEPTION_OBSTACLE_VISUALIZER_COMMON_BMP_H_
#define MODULES_PERCEPTION_OBSTACLE_VISUALIZER_COMMON_BMP_H_

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace apollo {
namespace perception {

struct BMPHeader {
  char bf_type[2];         /* "BM" */
  int bf_size;             /* Size of file in bytes */
  int bf_reserved;         /* set to 0 */
  int bf_off_bits;         /* Byte offset to actual bitmap data (= 54) */
  int bi_size;             /* Size of BITMAPINFOHEADER, in bytes (= 40) */
  int bi_width;            /* Width of image, in pixels */
  int bi_height;           /* Height of images, in pixels */
  int bi_planes;           /* Number of planes in target device (set to 1) */
  int bi_bit_count;        /* Bits per pixel (24 in this case) */
  int bi_compression;      /* Type of compression (0 if no compression) */
  int bi_size_image;       /* Image size, in bytes (0 if no compression) */
  int bi_x_pels_per_meter; /* Resolution in pixels/meter of display device */
  int bi_y_pels_per_meter; /* Resolution in pixels/meter of display device */
  int bi_clr_used;         /* Number of colors in the color table (if 0, use */
                           /* maximum allowed by bi_bit_count) */
  int bi_clr_important;    /* Number of important colors.  If 0, all colors */
                           /* are important */
};

inline void save_rgba_image_to_bmp(const unsigned char* rgba_image, const int w,
                                   const int h, const std::string& fileName) {
  int bytes_per_line = 0;

  struct BMPHeader bmph;

  /* The length of each line must be a multiple of 4 bytes */
  int width = w;
  int height = h;
  int mod4 = (width * 3) % 4;
  if (mod4 == 0) {
    bytes_per_line = 3 * width;
  } else {
    bytes_per_line = 3 * width + 4 - mod4;
  }

  bmph.bf_type[0] = 'B';
  bmph.bf_type[1] = 'M';
  bmph.bf_off_bits = 54;
  bmph.bf_size = bmph.bf_off_bits + bytes_per_line * height;
  bmph.bf_reserved = 0;
  bmph.bi_size = 40;
  bmph.bi_width = width;
  bmph.bi_height = height;
  bmph.bi_planes = 1;
  bmph.bi_bit_count = 24;
  bmph.bi_compression = 0;
  bmph.bi_size_image = bytes_per_line * height;
  bmph.bi_x_pels_per_meter = 0;
  bmph.bi_y_pels_per_meter = 0;
  bmph.bi_clr_used = 0;
  bmph.bi_clr_important = 0;

  std::fstream fs;
  fs.open(fileName, std::fstream::in | std::fstream::binary);

  fs << bmph.bf_type;
  fs << bmph.bf_size;
  fs << bmph.bf_reserved;
  fs << bmph.bf_off_bits;
  fs << bmph.bi_size;
  fs << bmph.bi_width;
  fs << bmph.bi_height;
  fs << bmph.bi_planes;
  fs << bmph.bi_bit_count;
  fs << bmph.bi_compression;
  fs << bmph.bi_size_image;
  fs << bmph.bi_x_pels_per_meter;
  fs << bmph.bi_y_pels_per_meter;
  fs << bmph.bi_clr_used;
  fs << bmph.bi_clr_important;

  std::vector<unsigned char> line(bytes_per_line);

  int index = 0;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      fs << rgba_image[index];
      fs << rgba_image[index + 1];
      fs << rgba_image[index + 2];
      index += 4;
    }
  }

  fs.close();
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_VISUALIZER_COMMON_BMP_H_
