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

#include <QtGui/QImage>
#include <QtGui/QOpenGLBuffer>
#include <memory>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

class Texture {
 public:
  Texture(void);
  ~Texture() {
    if (data_) {
      delete[] data_;
      data_ = nullptr;
    }
  }

  bool isSizeChanged(void) const { return is_size_changed_; }
  bool isDirty(void) const { return is_dirty_; }
  void removeDirty(void) { is_size_changed_ = is_dirty_ = false; }
  void setDirty(void) { is_dirty_ = true; }
  void setSizeChanged(void) { is_size_changed_ = is_dirty_ = true; }

  GLsizei width(void) const { return image_width_; }
  GLsizei height(void) const { return image_height_; }
  GLenum texture_format(void) const { return texture_format_; }
  GLsizei data_size(void) const { return data_size_; }

  bool UpdateData(const QImage& img);
  bool UpdateData(const std::shared_ptr<const apollo::drivers::Image>&);
  const GLubyte* data(void) const { return data_; }

 private:
  struct {
    int : 30;
    bool is_size_changed_ : 1;
    bool is_dirty_ : 1;
  };
  GLenum texture_format_;
  GLsizei image_width_;
  GLsizei image_height_;
  GLsizei data_size_;
  GLubyte* data_;
};
