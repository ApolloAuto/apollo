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

#ifndef PLANE_H
#define PLANE_H

#include <memory>
#include "renderable_object.h"
#include "texture.h"

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

  explicit Plane(std::shared_ptr<Texture>& t = NullTextureObj);
  virtual ~Plane(void) { texture_.reset(); }

  void set_texture(std::shared_ptr<Texture>& t) {
    if (t != texture_) {
      texture_ = t;
    }
  }

  virtual GLenum GetPrimitiveType(void) const { return GL_QUADS; }
  GLsizei texWidth(void) const { return texture_->width(); }
  GLsizei texHeight(void) const { return texture_->height(); }

 protected:
  virtual bool FillVertexBuffer(GLfloat* pBuffer) override;
  virtual void Draw(void) override;
  virtual void SetupAllAttrPointer(void) override;

 private:
  GLuint texture_id_;
  std::shared_ptr<Texture> texture_;
};

#endif  // PLANE_H
