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

#include "texture.h"
#include <iostream>

Texture::Texture()
    : is_size_changed_(false),
      is_dirty_(false),
      texture_format_(0),
      image_width_(0),
      image_height_(0),
      data_size_(0),
      data_(nullptr) {}

bool Texture::UpdateData(const QImage &img) {
  if (data_size_ < img.byteCount()) {
    if (!data_) {
      delete data_;
    }

    data_ = new GLubyte[img.byteCount()];
    if (data_ == nullptr) {
      data_size_ = 0;
      return false;
    }
    data_size_ = img.byteCount();
    is_size_changed_ = true;
  }

  memcpy(data_, img.bits(), img.byteCount());
  is_dirty_ = true;

  image_height_ = img.height();
  image_width_ = img.width();

  texture_format_ = GL_RGBA;
  return true;
}
