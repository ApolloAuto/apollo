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

#include <QtWidgets/QDialog>

namespace Ui {
class SceneCameraDialog;
}

class QVector3D;

class SceneCameraDialog : public QDialog {
  Q_OBJECT

 public:
  explicit SceneCameraDialog(QWidget *parent = nullptr);
  ~SceneCameraDialog();

 signals:
  void resetcamera();

  void sensitivityChanged(float);
  void cameraTypeChanged(int);

  void xValueChanged(double);
  void yValueChanged(double);
  void zValueChanged(double);
  void yawValueChanged(double);
  void pitchValueChanged(double);
  void rollValueChanged(double);

 public slots:  // NOLINT
  void updateCameraAttitude(const QVector3D &);
  void updateCameraPos(const QVector3D &);

 private slots:  // NOLINT
  void OnStepSlideChanged(int v);
  void onCameraTypeChanged(int);

 private:
  Ui::SceneCameraDialog *ui;
};
