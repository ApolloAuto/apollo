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

#include "modules/tools/visualizer/msg_dialog.h"
#include "modules/tools/visualizer/ui_msg_dialog.h"

MessageDialog::MessageDialog(QWidget *parent)
    : QDialog(parent), ui_(new Ui::MessageDialog) {
  ui_->setupUi(this);
}

MessageDialog::~MessageDialog() { delete ui_; }

void MessageDialog::setMessage(const QString &msg) {
  ui_->msgLabel->setText(msg);
}
