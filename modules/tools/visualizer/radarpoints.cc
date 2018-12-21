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

#include "radarpoints.h"

#include <iostream>

RadarPoints::RadarPoints(
    const std::shared_ptr<QOpenGLShaderProgram>& shaderProgram)
    : RenderableObject(1, 3, shaderProgram),
      color_(1.0f, 0.0f, 0.0f),
      buffer_(nullptr) {}

bool RadarPoints::FillData(
    const std::shared_ptr<const apollo::drivers::RadarObstacles>& rawData) {
  bool ret = false;

  set_vertex_count(rawData->radar_obstacle_size());
  buffer_ = new GLfloat[vertex_count() * vertex_element_count()];

  if (buffer_) {
    GLfloat* ptr = buffer_;
    const ::google::protobuf::Map<::google::protobuf::int32,
                                  apollo::drivers::RadarObstacle>&
        radarObstacles = rawData->radar_obstacle();
    for (::google::protobuf::Map<::google::protobuf::int32,
                                 apollo::drivers::RadarObstacle>::const_iterator
             iter = radarObstacles.cbegin();
         iter != radarObstacles.cend(); ++iter, ptr += vertex_element_count()) {
      const apollo::common::Point2D& position =
          iter->second.absolute_position();

      ptr[0] = static_cast<float>(position.x());
      ptr[1] = static_cast<float>(position.y());
      ptr[2] = std::pow(10.0f, static_cast<float>(iter->second.rcs() / 20.0));
    }  // end for

    ret = true;
  }

  return ret;
}

bool RadarPoints::FillVertexBuffer(GLfloat* pBuffer) {
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
