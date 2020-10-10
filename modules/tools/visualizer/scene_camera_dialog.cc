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

#include "modules/tools/visualizer/scene_camera_dialog.h"

#include <QtGui/QVector3D>

#include "modules/tools/visualizer/ui_scene_camera_dialog.h"

SceneCameraDialog::SceneCameraDialog(QWidget* parent)
    : QDialog(parent), ui(new Ui::SceneCameraDialog) {
  ui->setupUi(this);
  ui->cameraX->setEnabled(false);
  ui->cameraY->setEnabled(false);
  ui->cameraZ->setEnabled(false);

  connect(ui->resetButton, SIGNAL(clicked()), this, SIGNAL(resetcamera()));
  connect(ui->cameraTypeComboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(onCameraTypeChanged(int)));
  connect(ui->cameraX, SIGNAL(valueChanged(double)), this,
          SIGNAL(xValueChanged(double)));
  connect(ui->cameraY, SIGNAL(valueChanged(double)), this,
          SIGNAL(yValueChanged(double)));
  connect(ui->cameraZ, SIGNAL(valueChanged(double)), this,
          SIGNAL(zValueChanged(double)));
  connect(ui->cameraYaw, SIGNAL(valueChanged(double)), this,
          SIGNAL(yawValueChanged(double)));
  connect(ui->cameraPitch, SIGNAL(valueChanged(double)), this,
          SIGNAL(pitchValueChanged(double)));
  connect(ui->cameraRoll, SIGNAL(valueChanged(double)), this,
          SIGNAL(rollValueChanged(double)));
  connect(ui->stepSlider, SIGNAL(valueChanged(int)), this,
          SLOT(OnStepSlideChanged(int)));
}

SceneCameraDialog::~SceneCameraDialog() { delete ui; }

void SceneCameraDialog::updateCameraAttitude(const QVector3D& attitude) {
  ui->cameraYaw->setValue(attitude.x());
  ui->cameraPitch->setValue(attitude.y());
  ui->cameraRoll->setValue(attitude.z());
}

void SceneCameraDialog::updateCameraPos(const QVector3D& pos) {
  ui->cameraX->setValue(pos.x());
  ui->cameraY->setValue(pos.y());
  ui->cameraZ->setValue(pos.z());
}

void SceneCameraDialog::OnStepSlideChanged(int v) {
  const float step =
      static_cast<float>(v) / static_cast<float>(ui->stepSlider->maximum());

  emit sensitivityChanged(step);

  ui->cameraX->setSingleStep(step);
  ui->cameraY->setSingleStep(step);
  ui->cameraZ->setSingleStep(step);

  ui->cameraYaw->setSingleStep(step);
  ui->cameraPitch->setSingleStep(step);
  ui->cameraRoll->setSingleStep(step);
}

void SceneCameraDialog::onCameraTypeChanged(int index) {
  emit cameraTypeChanged(index);
  ui->cameraX->setEnabled(index);
  ui->cameraY->setEnabled(index);
  ui->cameraZ->setEnabled(index);
}
