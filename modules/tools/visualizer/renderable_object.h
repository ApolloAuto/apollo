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

#include <QtGui/QOpenGLBuffer>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLVertexArrayObject>
#include <memory>

class RenderableObject : protected QOpenGLFunctions {
 public:
  static std::shared_ptr<QOpenGLShaderProgram> NullRenderableObj;
  static std::shared_ptr<QOpenGLShaderProgram> CreateShaderProgram(
      const QString& vertexShaderFileName, const QString& fragShaderFileName);

  explicit RenderableObject(int vertex_count = 1, int vertex_element_count = 3,
                            const std::shared_ptr<QOpenGLShaderProgram>&
                                shaderProgram = NullRenderableObj);
  virtual ~RenderableObject(void);

  virtual GLenum GetPrimitiveType(void) const = 0;
  virtual void SetupExtraUniforms(void) {}

  bool is_renderable(void) const { return is_renderable_; }
  void set_is_renderable(bool b) { is_renderable_ = b; }

  int vertex_count(void) const { return vertex_count_; }
  void set_vertex_count(int vertexCount) { vertex_count_ = vertexCount; }

  int vertex_element_count(void) const { return vertex_element_count_; }
  void set_vertex_element_count(int vertexElementCount) {
    vertex_element_count_ = vertexElementCount;
  }

  int VertexBufferSize(void) const {
    return vertex_count() * vertex_element_count() *
           static_cast<int>(sizeof(GLfloat));
  }

  void set_shader_program(
      const std::shared_ptr<QOpenGLShaderProgram>& shaderProgram) {
    shader_program_ = shaderProgram;
  }

  bool haveShaderProgram(void) { return shader_program_ != nullptr; }

  bool Init(
      std::shared_ptr<QOpenGLShaderProgram>& shaderProgram =
          NullRenderableObj);  // initial vao, vbo and call fillVertexBuffer
  void Destroy(void);

  void Render(const QMatrix4x4* mvp = nullptr);

 protected:
  virtual bool FillVertexBuffer(GLfloat* pBuffer) = 0;
  virtual void Draw(void) {
    glDrawArrays(GetPrimitiveType(), 0, vertex_count());
  }

  virtual void SetupAllAttrPointer(void) {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, vertex_element_count(), GL_FLOAT, GL_FALSE,
        static_cast<int>(sizeof(GLfloat)) * vertex_element_count(), 0);
  }

  bool is_init_;
  bool is_renderable_;

  int vertex_count_;
  int vertex_element_count_;

  std::shared_ptr<QOpenGLShaderProgram> shader_program_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer vbo_;
};
