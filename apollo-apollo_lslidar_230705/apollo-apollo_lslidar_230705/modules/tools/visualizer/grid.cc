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

#include "modules/tools/visualizer/grid.h"

Grid::Grid(int cellCountBySide)
    : RenderableObject((cellCountBySide << 2) + 4, 2),
      grid_color_(128, 128, 128) {}

bool Grid::FillVertexBuffer(GLfloat* pBuffer) {
  float x = static_cast<float>(CellCount()) / 2.0f;
  float z;

  for (z = -x; z <= x; ++z) {
    *pBuffer++ = -x;
    *pBuffer++ = z;
    *pBuffer++ = x;
    *pBuffer++ = z;
  }

  z = x;

  for (x = -z; x <= z; ++x) {
    *pBuffer++ = x;
    *pBuffer++ = -z;
    *pBuffer++ = x;
    *pBuffer++ = z;
  }

  return true;
}
