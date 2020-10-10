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

#include <QtWidgets/QOpenGLWidget>
#include <memory>

#include "modules/tools/visualizer/free_camera.h"
#include "modules/tools/visualizer/plane.h"

class Texture;
class FixedAspectRatioWidget;

class VideoImgViewer : public QOpenGLWidget, protected QOpenGLFunctions {
 public:
  explicit VideoImgViewer(QWidget* parent = nullptr);
  ~VideoImgViewer();

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

 private:
  bool is_init_;
  int mvp_id_;

  Plane plane_;
  FreeCamera ortho_camera_;
  std::shared_ptr<Texture> default_image_;

  std::shared_ptr<QOpenGLShaderProgram> video_image_shader_prog_;
  friend class FixedAspectRatioWidget;
};
