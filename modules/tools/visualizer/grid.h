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

#include <QtGui/QColor>

#include "modules/tools/visualizer/renderable_object.h"

class Grid : public RenderableObject {
 public:
  explicit Grid(int cellCountBySide = 1);
  ~Grid() {}

  GLenum GetPrimitiveType(void) const { return GL_LINES; }
  void SetupExtraUniforms(void) {
    QVector3D color;
    color.setX(static_cast<float>(grid_color_.redF()));
    color.setY(static_cast<float>(grid_color_.greenF()));
    color.setZ(static_cast<float>(grid_color_.blueF()));
    shader_program_->setUniformValue("color", color);
  }

  int red(void) const { return grid_color_.red(); }
  int green(void) const { return grid_color_.green(); }
  int blue(void) const { return grid_color_.blue(); }

  const QColor& grid_color(void) const { return grid_color_; }
  void set_grid_color(const QColor& color) { grid_color_ = color; }

  void SetCellCount(int cellCount) {
    set_vertex_count(((cellCount << 1) + 2) * vertex_element_count());
  }

  int CellCount(void) const {
    return (vertex_count() / vertex_element_count() - 2) >> 1;
  }

 protected:
  virtual bool FillVertexBuffer(GLfloat* pBuffer);

 private:
  QColor grid_color_;
};
