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

#ifndef TOOLS_CVT_VISUALIZER_SCENEVIEWER_H_
#define TOOLS_CVT_VISUALIZER_SCENEVIEWER_H_

#include "free_camera.h"
#include "plane.h"
#include "renderable_object.h"
#include "target_camera.h"

#include <QList>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <map>
#include <memory>

class QTimer;
class SceneCameraDialog;

class SceneViewer : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT
 public:
  enum CameraType {
    TARGET,
    FREE,
  };

  explicit SceneViewer(QWidget* parent = nullptr);
  ~SceneViewer();

  bool is_init(void) const { return is_init_; }
  bool IsFreeCamera(void) const { return current_cameraPtr_ == &free_camera_; }

  bool AddTempRenderableObj(RenderableObject* renderObj) {
    if (renderObj && renderObj->haveShaderProgram() && is_init_) {
      tmp_renderable_obj_list_.append(renderObj);
      return true;
    } else
      return false;
  }

  bool AddPermanentRenderObj(RenderableObject* obj) {
    if (obj && obj->haveShaderProgram() && is_init_) {
      permanent_renderable_obj_list_.append(obj);
      return true;
    } else {
      return false;
    }
  }

  const QVector3D& CameraPos(void) const {
    return current_cameraPtr_->position();
  }
  const QVector3D& CamerAttitude(void) const {
    return current_cameraPtr_->attitude();
  }

  float sensitivity(void) const { return sensitivity_; }

  void AddNewShaderProg(const std::string& shaderProgName,
                        std::shared_ptr<QOpenGLShaderProgram>& newShaderProg) {
    if (newShaderProg == nullptr) {
      return;
    }

    auto iter = managed_shader_prog_.find(shaderProgName);
    if (iter != managed_shader_prog_.end()) {
      iter->second.reset();
    }

    managed_shader_prog_[shaderProgName] = newShaderProg;
  }

  std::shared_ptr<QOpenGLShaderProgram> FindShaderProg(
      const std::string& shaderProgName) {
    if (managed_shader_prog_.find(shaderProgName) !=
        managed_shader_prog_.end()) {
      return managed_shader_prog_[shaderProgName];
    } else {
      return std::shared_ptr<QOpenGLShaderProgram>();
    }
  }

 signals:
  void CameraPosChanged(const QVector3D& pos);
  void CameraAttitudeChanged(const QVector3D& attitude);

 public slots:
  void ChangeCameraType(int index);
  //  void ChangeCameraMode(int index);

  void ResetCameraPosAttitude(void);

  void UpdateCameraX(double x);
  void UpdateCameraY(double Y);
  void UpdateCameraZ(double Z);

  void UpdateCameraYaw(double yawInDegrees);
  void UpdateCameraPitch(double pitchInDegrees);
  void UpdateCameraRoll(double rollInDegrees);

  void set_sensitivity(float s) { sensitivity_ = s; }

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void enterEvent(QEvent* event) override;
  void leaveEvent(QEvent* event) override;

  void mousePressEvent(QMouseEvent* mouseEvent) override;
  void mouseMoveEvent(QMouseEvent* mouseEvent) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  void UpdateCameraWorld(void);
  void UpdateAllShaderProgMVP(const QMatrix4x4& mvp);

  bool is_init_;
  bool right_key_is_moved_;

  float sensitivity_;
  QTimer* refreshTimer_;

  QPoint left_key_last_pos_;
  float right_key_last_y_;

  SceneCameraDialog* camera_dialog_;
  AbstractCamera* current_cameraPtr_;

  FreeCamera free_camera_;
  TargetCamera target_camera_;

  std::map<const std::string, std::shared_ptr<QOpenGLShaderProgram>>
      managed_shader_prog_;

  QList<RenderableObject*> tmp_renderable_obj_list_;
  QList<RenderableObject*> permanent_renderable_obj_list_;
};

#endif  // TOOLS_CVT_VISUALIZER_SCENEVIEWER_H_
