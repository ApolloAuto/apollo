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
class MessageDialog;
}

class MessageDialog : public QDialog {
  Q_OBJECT

 public:
  explicit MessageDialog(QWidget* parent = nullptr);
  ~MessageDialog();

  void setMessage(const QString& msg);
  void setMessage(const char* msg) { setMessage(QString(msg)); }

 private:
  Ui::MessageDialog* ui_;
};
