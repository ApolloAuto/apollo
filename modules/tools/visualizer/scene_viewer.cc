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

#include "modules/tools/visualizer/scene_viewer.h"

#include <QtCore/QTimer>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QMessageBox>

#include "modules/tools/visualizer/scene_camera_dialog.h"

struct SceneViewer::TempRenderableObjGroup {
  bool isEnabled_;
  QList<RenderableObject *> objGroupList_;

  TempRenderableObjGroup(void) : isEnabled_(true), objGroupList_() {}
  ~TempRenderableObjGroup(void) {
    while (!objGroupList_.isEmpty()) {
      delete objGroupList_.takeFirst();
    }

    objGroupList_.clear();
  }

  bool isEmpty(void) const { return objGroupList_.isEmpty(); }

  bool isEnabled(void) const { return isEnabled_; }

  void append(RenderableObject *obj) { objGroupList_.append(obj); }
  RenderableObject *takeFirst(void) { return objGroupList_.takeFirst(); }
  RenderableObject *takeLast(void) { return objGroupList_.takeLast(); }
};

SceneViewer::SceneViewer(QWidget *parent)
    : QOpenGLWidget(parent),
      QOpenGLFunctions(),
      is_init_(false),
      right_key_is_moved_(false),
      sensitivity_(0.08f),
      refreshTimer_(nullptr),
      left_key_last_pos_(),
      right_key_last_y_(0),
      camera_dialog_(nullptr),
      current_cameraPtr_(nullptr),
      free_camera_(),
      target_camera_(),
      managed_shader_prog_(),
      tmp_renderable_obj_list_(),
      permanent_renderable_obj_list_() {
  current_cameraPtr_ = &target_camera_;
}

SceneViewer::~SceneViewer() {
  if (is_init_) {
    is_init_ = false;

    refreshTimer_->stop();
    delete refreshTimer_;

    makeCurrent();

    auto iter = managed_shader_prog_.begin();
    for (; iter != managed_shader_prog_.end(); ++iter) {
      iter->second->destroyed();
      iter->second.reset();
    }

    managed_shader_prog_.clear();

    for (auto &iter : tmp_renderable_obj_list_) {
      delete iter.second;
    }

    tmp_renderable_obj_list_.clear();

    while (!permanent_renderable_obj_list_.isEmpty()) {
      delete permanent_renderable_obj_list_.takeFirst();
    }

    doneCurrent();
  }
}

void SceneViewer::setTempObjGroupEnabled(const std::string &tmpObjGroupName,
                                         bool b) {
  auto iter = tmp_renderable_obj_list_.find(tmpObjGroupName);
  if (iter != tmp_renderable_obj_list_.cend()) {
    iter->second->isEnabled_ = b;
  }
}

bool SceneViewer::AddTempRenderableObj(const std::string &tmpObjGroupName,
                                       RenderableObject *renderObj) {
  bool ret = false;
  if (renderObj && renderObj->haveShaderProgram() && is_init_) {
    auto iter = tmp_renderable_obj_list_.find(tmpObjGroupName);
    if (iter == tmp_renderable_obj_list_.cend()) {
      TempRenderableObjGroup *objGroup = new TempRenderableObjGroup();
      if (objGroup) {
        objGroup->append(renderObj);
        tmp_renderable_obj_list_.insert(
            std::make_pair(tmpObjGroupName, objGroup));
        ret = true;
      }
    } else {
      iter->second->objGroupList_.append(renderObj);
      ret = true;
    }
  }

  return ret;
}

void SceneViewer::AddNewShaderProg(
    const std::string &shaderProgName,
    const std::shared_ptr<QOpenGLShaderProgram> &newShaderProg) {
  if (newShaderProg == nullptr) {
    return;
  }

  auto iter = managed_shader_prog_.find(shaderProgName);
  if (iter != managed_shader_prog_.end()) {
    iter->second.reset();
  }

  QMatrix4x4 mvp = current_cameraPtr_->projection_matrix() *
                   current_cameraPtr_->model_view_matrix();

  newShaderProg->bind();
  newShaderProg->setUniformValue("mvp", mvp);
  newShaderProg->release();

  managed_shader_prog_[shaderProgName] = newShaderProg;
}

void SceneViewer::initializeGL() {
  initializeOpenGLFunctions();

  glPointSize(1.0f);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  refreshTimer_ = new QTimer(this);
  if (!refreshTimer_) {
    return;
  }

  refreshTimer_->setObjectName(tr("_refreshTimer"));
  refreshTimer_->setInterval(40);
  refreshTimer_->start();
  connect(refreshTimer_, SIGNAL(timeout()), this, SLOT(repaint()));

  free_camera_.SetUpProjection(45.0f, GLfloat(width()), GLfloat(height()));
  target_camera_.SetUpProjection(45.0f, GLfloat(width()), GLfloat(height()));
  ResetCameraPosAttitude();

  is_init_ = true;
}

void SceneViewer::resizeGL(int width, int height) {
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);

  if (is_init_) {
    free_camera_.set_near_plane_width(GLfloat(width));
    free_camera_.set_near_plane_height(GLfloat(height));
    free_camera_.UpdateProjection();

    target_camera_.set_near_plane_width(GLfloat(width));
    target_camera_.set_near_plane_height(GLfloat(height));
    target_camera_.UpdateProjection();

    QMatrix4x4 mvp = current_cameraPtr_->projection_matrix() *
                     current_cameraPtr_->model_view_matrix();

    UpdateAllShaderProgMVP(mvp);
  }
}

void SceneViewer::paintGL() {
  if (is_init_) {
    for (RenderableObject *item : permanent_renderable_obj_list_) {
      if (item->Init()) {
        item->Render();
      }
    }

    for (auto &iter : tmp_renderable_obj_list_) {
      if (!iter.second->isEmpty()) {
        RenderableObject *lastItem = nullptr;

        if (iter.second->isEnabled()) {
          lastItem = iter.second->takeLast();
          if (lastItem->Init()) {
            lastItem->Render();
          }
        }

        while (!iter.second->isEmpty()) {
          delete iter.second->takeFirst();
        }

        if (lastItem) {
          iter.second->append(lastItem);
        }
      }
    }
  }
}

void SceneViewer::ChangeCameraType(int index) {
  if (index < TARGET || index > FREE) {
    return;
  }
  if (index == FREE) {
    current_cameraPtr_ = &free_camera_;
  } else {
    current_cameraPtr_ = &target_camera_;
  }
  ResetCameraPosAttitude();
}

void SceneViewer::ResetCameraPosAttitude(void) {
  free_camera_.set_position(0.0f, 48.0f, -48.0f);
  free_camera_.SetAttitude(0.0f, 45.0f, 0.0f);

  target_camera_.set_position(-21.0f, 21.0f, 0.0f);
  target_camera_.Rotate(-45.0f, -90.0f, 0.0f);
  target_camera_.set_target_pos(0.0f, 0.0f, 0.0f);

  emit CameraPosChanged(current_cameraPtr_->position());
  emit CameraAttitudeChanged(current_cameraPtr_->attitude());

  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraX(double x) {
  free_camera_.set_x(static_cast<float>(x));
  target_camera_.set_x(static_cast<float>(x));
  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraY(double y) {
  free_camera_.set_y(static_cast<float>(y));
  target_camera_.set_y(static_cast<float>(y));
  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraZ(double z) {
  free_camera_.set_z(static_cast<float>(z));
  target_camera_.set_y(static_cast<float>(z));
  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraYaw(double yawInDegrees) {
  if (yawInDegrees > 360.0) {
    yawInDegrees -= 360.0;
  }
  if (yawInDegrees < -360.0) {
    yawInDegrees += 360.0;
  }

  free_camera_.set_yaw(static_cast<float>(yawInDegrees));
  target_camera_.set_yaw(static_cast<float>(yawInDegrees));
  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraPitch(double pitchInDegrees) {
  if (pitchInDegrees > 90.0) {
    pitchInDegrees = 90.0;
  }
  if (pitchInDegrees < -90.0) {
    pitchInDegrees = -90.0;
  }

  free_camera_.set_pitch(static_cast<float>(pitchInDegrees));
  target_camera_.set_pitch(static_cast<float>(pitchInDegrees));
  UpdateCameraWorld();
}
void SceneViewer::UpdateCameraRoll(double rollInDegrees) {
  if (rollInDegrees > 360.0) {
    rollInDegrees -= 360.0;
  }
  if (rollInDegrees < -360.0) {
    rollInDegrees += 360.0;
  }

  free_camera_.set_roll(static_cast<float>(rollInDegrees));
  free_camera_.set_roll(static_cast<float>(rollInDegrees));
  UpdateCameraWorld();
}

void SceneViewer::UpdateCameraWorld(void) {
  free_camera_.UpdateWorld();
  target_camera_.UpdateWorld();

  QMatrix4x4 mvp = current_cameraPtr_->projection_matrix() *
                   current_cameraPtr_->model_view_matrix();

  UpdateAllShaderProgMVP(mvp);

  update();
}

void SceneViewer::UpdateAllShaderProgMVP(const QMatrix4x4 &mvp) {
  for (auto iter = managed_shader_prog_.begin();
       iter != managed_shader_prog_.end(); ++iter) {
    iter->second->bind();
    iter->second->setUniformValue("mvp", mvp);
    iter->second->release();
  }
}

void SceneViewer::enterEvent(QEvent *event) {
  setCursor(Qt::SizeAllCursor);
  QOpenGLWidget::enterEvent(event);
}

void SceneViewer::leaveEvent(QEvent *event) {
  unsetCursor();
  QOpenGLWidget::leaveEvent(event);
}

void SceneViewer::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    left_key_last_pos_ = QCursor::pos();
  } else if (event->button() == Qt::RightButton) {
    right_key_is_moved_ = false;
    right_key_last_y_ = static_cast<float>(QCursor::pos().y());
  }

  QOpenGLWidget::mousePressEvent(event);
}

void SceneViewer::mouseMoveEvent(QMouseEvent *mouseEvent) {
  if (mouseEvent->buttons() == Qt::LeftButton) {
    QPoint tmp = QCursor::pos();
    QPoint dd = tmp - left_key_last_pos_;
    left_key_last_pos_ = tmp;

    float x = static_cast<float>(dd.x()) * sensitivity();
    float y = static_cast<float>(dd.y()) * sensitivity();

    if (IsFreeCamera()) {
      free_camera_.Starfe(-x);
      free_camera_.Lift(y);

      emit CameraPosChanged(free_camera_.position());
    } else {
      x = target_camera_.yaw() - x;
      y = target_camera_.pitch() - y;

      if (x > 360.0f) {
        x -= 360.0f;
      }
      if (x < -360.0f) {
        x += 360.0f;
      }

      if (y > 90.0f) {
        y = 90.0f;
      }
      if (y < -90.0f) {
        y = 90.0f;
      }

      target_camera_.set_yaw(x);
      target_camera_.set_pitch(y);

      emit CameraAttitudeChanged(target_camera_.attitude());
    }

    goto _label1;
  } else if (mouseEvent->buttons() == Qt::RightButton) {
    right_key_is_moved_ = true;
    if (IsFreeCamera()) {
      {
        float tmp = static_cast<float>(QCursor::pos().y());
        float dd = right_key_last_y_ - tmp;
        right_key_last_y_ = tmp;

        float z = dd * sensitivity();

        free_camera_.Walk(z);

        emit CameraPosChanged(free_camera_.position());
      }

    _label1:
      UpdateCameraWorld();
      update();
    }
  }
  QOpenGLWidget::mousePressEvent(mouseEvent);
}

void SceneViewer::wheelEvent(QWheelEvent *event) {
  float delta = static_cast<float>(event->angleDelta().y());
  delta *= sensitivity();

  target_camera_.set_distance(target_camera_.distance() + delta);

  delta += free_camera_.pitch();
  if (delta > 90.0f) {
    delta = 90.0f;
  }
  if (delta < -90.0f) {
    delta = -90.0f;
  }
  free_camera_.set_pitch(delta);

  if (IsFreeCamera()) {
    emit CameraAttitudeChanged(free_camera_.attitude());
  } else {
    emit CameraPosChanged(target_camera_.position());
  }
  UpdateCameraWorld();
  update();

  QOpenGLWidget::wheelEvent(event);
}

void SceneViewer::mouseReleaseEvent(QMouseEvent *event) {
  if (!right_key_is_moved_ && event->button() == Qt::RightButton) {
    if (camera_dialog_ == nullptr) {
      camera_dialog_ = new SceneCameraDialog(this);
      if (!camera_dialog_) {
        QMessageBox::warning(this, tr("Error"),
                             tr("No Enough for creating Camera Dialog!!!"),
                             QMessageBox::Ok);
        goto retLabel;
      } else {
        connect(camera_dialog_, SIGNAL(resetcamera()), this,
                SLOT(ResetCameraPosAttitude()));
        connect(camera_dialog_, SIGNAL(xValueChanged(double)), this,
                SLOT(UpdateCameraX(double)));
        connect(camera_dialog_, SIGNAL(yValueChanged(double)), this,
                SLOT(UpdateCameraY(double)));
        connect(camera_dialog_, SIGNAL(zValueChanged(double)), this,
                SLOT(UpdateCameraZ(double)));
        connect(camera_dialog_, SIGNAL(yawValueChanged(double)), this,
                SLOT(UpdateCameraYaw(double)));
        connect(camera_dialog_, SIGNAL(pitchValueChanged(double)), this,
                SLOT(UpdateCameraPitch(double)));
        connect(camera_dialog_, SIGNAL(rollValueChanged(double)), this,
                SLOT(UpdateCameraRoll(double)));
        connect(camera_dialog_, SIGNAL(cameraTypeChanged(int)), this,
                SLOT(ChangeCameraType(int)));
        connect(camera_dialog_, SIGNAL(sensitivityChanged(float)), this,
                SLOT(set_sensitivity(float)));

        connect(this, SIGNAL(CameraAttitudeChanged(QVector3D)), camera_dialog_,
                SLOT(updateCameraAttitude(QVector3D)));
        connect(this, SIGNAL(CameraPosChanged(QVector3D)), camera_dialog_,
                SLOT(updateCameraPos(QVector3D)));
      }
    }
    camera_dialog_->updateCameraPos(current_cameraPtr_->position());
    camera_dialog_->updateCameraAttitude(current_cameraPtr_->attitude());

    camera_dialog_->move(event->globalPos());
    camera_dialog_->show();
  }

retLabel:
  QOpenGLWidget::mouseReleaseEvent(event);
}
