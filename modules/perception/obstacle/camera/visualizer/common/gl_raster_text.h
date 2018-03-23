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

#ifndef MODULES_PERCEPTION_OBSTACLE_VISUALIZER_GL_RASTER_TEXT_H_
#define MODULES_PERCEPTION_OBSTACLE_VISUALIZER_GL_RASTER_TEXT_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <boost/shared_ptr.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

class GLRasterText {
 public:
  GLRasterText() {}

  ~GLRasterText() {}

  void init();

  void print_string(const char *s);

 private:
  void make_raster_font();

  static GLubyte s_space_bitmap_[];
  static GLubyte s_letters_bitmaps_[][13];
  static GLubyte s_numbers_bitmaps_[][13];
  static GLubyte s_asccii_bitmaps_[][13];
  static GLuint s_font_offset_;
};

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_GL_RASTER_TEXT_H
