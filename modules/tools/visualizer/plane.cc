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

#include "modules/tools/visualizer/plane.h"

std::shared_ptr<Texture> Plane::NullTextureObj;

Plane::Plane(const std::shared_ptr<Texture>& t)
    : RenderableObject(4, 4), texture_id_(0), texture_(t) {}

bool Plane::FillVertexBuffer(GLfloat* pBuffer) {
  if (texture_ == nullptr || !texture_->isDirty()) {
    return false;
  }
  glGenTextures(1, &texture_id_);

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texture_->width(),
               texture_->height(), 0, texture_->texture_format(),
               GL_UNSIGNED_BYTE, texture_->data());

  glBindTexture(GL_TEXTURE_2D, 0);

  texture_->removeDirty();

  pBuffer[0] = -1.0f;
  pBuffer[1] = -1.0f;
  pBuffer[2] = 0.0f;
  pBuffer[3] = 0.0f;
  pBuffer[4] = 1.0f;
  pBuffer[5] = -1.0f;
  pBuffer[6] = 1.0f;
  pBuffer[7] = 0.0f;
  pBuffer[8] = 1.0f;
  pBuffer[9] = 1.0f;
  pBuffer[10] = 1.0f;
  pBuffer[11] = 1.0f;
  pBuffer[12] = -1.0f;
  pBuffer[13] = 1.0f;
  pBuffer[14] = 0.0f;
  pBuffer[15] = 1.0f;

  return true;
}

void Plane::SetupAllAttrPointer(void) {
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(
      0, 2, GL_FLOAT, GL_FALSE,
      static_cast<int>(sizeof(GLfloat)) * vertex_element_count(), 0);

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1, 2, GL_FLOAT, GL_FALSE,
      static_cast<int>(sizeof(GLfloat)) * vertex_element_count(),
      reinterpret_cast<void*>(sizeof(GLfloat) * 2));
}

void Plane::Draw(void) {
  if (texture_->data()) {
    if (texture_->isSizeChanged()) {
      glGenTextures(1, &texture_id_);

      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texture_->width(),
                   texture_->height(), 0, texture_->texture_format(),
                   GL_UNSIGNED_BYTE, texture_->data());

      glBindTexture(GL_TEXTURE_2D, 0);

      texture_->removeDirty();
    } else if (texture_->isDirty()) {
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_->width(),
                      texture_->height(), texture_->texture_format(),
                      GL_UNSIGNED_BYTE, texture_->data());
      glBindTexture(GL_TEXTURE_2D, 0);
      texture_->removeDirty();
    }
  }

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  RenderableObject::Draw();
  glBindTexture(GL_TEXTURE_2D, 0);
}
