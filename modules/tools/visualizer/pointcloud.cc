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

#include "modules/tools/visualizer/pointcloud.h"

#include <iostream>

PointCloud::PointCloud(
    int pointCount, int vertexElementCount,
    const std::shared_ptr<QOpenGLShaderProgram>& shaderProgram)
    : RenderableObject(pointCount, vertexElementCount, shaderProgram),
      buffer_(nullptr) {}

PointCloud::~PointCloud(void) {
  if (buffer_) {
    delete[] buffer_;
    buffer_ = nullptr;
  }
}

bool PointCloud::FillVertexBuffer(GLfloat* pBuffer) {
  if (buffer_ && pBuffer) {
    memcpy(pBuffer, buffer_, VertexBufferSize());
    delete[] buffer_;
    buffer_ = nullptr;
    return true;
  } else {
    std::cout << "---Error!!! cannot upload data to Graphics Card----"
              << std::endl;
    return false;
  }
}

bool PointCloud::FillData(
    const std::shared_ptr<const apollo::drivers::PointCloud>& pdata) {
  assert(vertex_count() == pdata->point_size());
  buffer_ = new GLfloat[vertex_count() * vertex_element_count()];
  if (buffer_) {
    GLfloat* tmp = buffer_;

    for (int i = 0; i < vertex_count(); ++i, tmp += vertex_element_count()) {
      const apollo::drivers::PointXYZIT& point = pdata->point(i);
      tmp[0] = point.x();
      tmp[1] = point.z();
      tmp[2] = -point.y();
      tmp[3] = static_cast<float>(point.intensity());
    }
    return true;
  }
  return false;
}
