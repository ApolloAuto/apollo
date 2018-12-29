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

#include "modules/tools/visualizer/renderable_object.h"
#include "modules/tools/visualizer/texture.h"

class Plane : public RenderableObject {
  /*
   *  struct Vertex
   *  {
   *      GLfloat _vert[2];
   *      GLfloat _texCoor[2];
   *  };
   *
   */

 public:
  static std::shared_ptr<Texture> NullTextureObj;

  explicit Plane(const std::shared_ptr<Texture>& t = NullTextureObj);
  virtual ~Plane(void) { texture_.reset(); }

  void set_texture(const std::shared_ptr<Texture>& t) {
    if (t != texture_) {
      texture_ = t;
    }
  }

  GLenum GetPrimitiveType(void) const override { return GL_QUADS; }
  GLsizei texWidth(void) const { return texture_->width(); }
  GLsizei texHeight(void) const { return texture_->height(); }

 protected:
  bool FillVertexBuffer(GLfloat* pBuffer) override;
  void Draw(void) override;
  void SetupAllAttrPointer(void) override;

 private:
  GLuint texture_id_;
  std::shared_ptr<Texture> texture_;
};
