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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MessageDialog;
class LoaderThread;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  void openPerfData(const QString &filePathName);

 protected:
  bool event(QEvent *);

 private slots:
  void actioOpenPerfData(void);
  void showHelpMenuInfo(void);
  void showTimeGrid(bool b);
  void handleSparerResult(void);
  void showDataBlock(int index);

 private:
  Ui::MainWindow *ui;
  MessageDialog *_msgDialog;
  LoaderThread *_loaderThread;
  int _maxDataBlockIndex;
};

#endif  // MAINWINDOW_H
