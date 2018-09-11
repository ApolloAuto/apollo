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

#include "mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QWheelEvent>
#include <iostream>
#include "msg_dialog.h"
#include "time_block_item.h"
#include "time_line_scene.h"
#include "time_line_widget.h"
#include "ui_mainwindow.h"

namespace {
const char* aboutMessage =
    "Cyber_VPerf\n"
    "\n"
    "One Visualization Tool for Presenting Cybertron Perf Data\n";

const char* licenseMessage =
    "Copyright 2018 The Apollo Authors. All Rights Reserved.\n"
    "\n"
    "Licensed under the Apache License, Version 2.0 (the \"License\");\n"
    "you may not use this file except in compliance with the License.\n"
    "You may obtain a copy of the License at\n"
    "\n"
    "http://www.apache.org/licenses/LICENSE-2.0\n"
    "\n"
    "Unless required by applicable law or agreed to in writing, software\n"
    "distributed under the License is distributed on an \"AS IS\" BASIS,\n"
    "WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
    "See the License for the specific language governing permissions and\n"
    "limitations under the License.\n";
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      _msgDialog(new MessageDialog) {
  ui->setupUi(this);

  ui->mainToolBar->addAction(ui->actionOpen);
  ui->mainToolBar->addSeparator();
  ui->mainToolBar->addAction(ui->actionFindTimeLength);
  ui->mainToolBar->addAction(ui->actionFindTimePoint);

  ui->axisScaleSpinBox->setValue(ui->timeLineWidget->axisScale());
  ui->axisSparsitySpinBox->setValue(ui->timeLineWidget->axisSparsity());

  connect(ui->actionOpen, SIGNAL(triggered(bool)), this,
          SLOT(actioOpenPerfData()));

  connect(ui->axisScaleSpinBox, SIGNAL(valueChanged(int)), ui->timeLineWidget,
          SLOT(setSceneScale(int)));
  connect(ui->axisSparsitySpinBox, SIGNAL(valueChanged(int)),
          ui->timeLineWidget, SLOT(setSceneSparsity(int)));

  connect(ui->actionFindTimeLength, SIGNAL(triggered(bool)), ui->timeLineWidget,
          SLOT(showFindWindow()));
  connect(ui->actionFindTimePoint, SIGNAL(triggered(bool)), ui->timeLineWidget,
          SLOT(showFindTimePointWindow()));

  connect(ui->timeGridPushButton, SIGNAL(clicked(bool)), this,
          SLOT(showTimeGrid(bool)));
}

MainWindow::~MainWindow() {
  delete ui;
  delete _msgDialog;
}

void MainWindow::showTimeGrid(bool b) {
  ui->timeLineWidget->showTimeGrid(b);
  if (b) {
    ui->timeGridPushButton->setText(tr("Hide"));
  } else {
    ui->timeGridPushButton->setText(tr("Show"));
  }
}

void MainWindow::actioOpenPerfData(void) {
  openPerfData(QFileDialog::getOpenFileName(this, tr("Open VPerf Data")));
}

void MainWindow::openPerfData(const QString& fileName) {
  if (fileName.isEmpty()) {
    return;
  }

  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::information(
        nullptr, tr("Parse VPerfData"),
        tr("Cannot open fille (%1)").arg(/*_currentFile*/ fileName));
    return;
  }

  QTextStream in(&file);
  QString line = in.readLine();

  int64_t tmp = line.toLongLong();
  ui->timeBaseContentLabel->setText(line);
  ui->timeLineWidget->setTimeBegin(tmp);

  tmp = 0;

  QStringList strList;
  line = in.readLine();
  while (!line.isNull()) {
    strList = line.split('\t');
    if (strList.count() == 7) {
      int64_t s = strList.at(5).toLongLong();
      int64_t e = strList.at(6).toLongLong();

      if (e > tmp) {
        tmp = e;
      }

      TimeBlockItem* blockItem = ui->timeLineWidget->addTimeBlock(
          strList.at(3).toInt(), strList.at(2), s, e);
      if (blockItem) {
        blockItem->setEventId(strList.at(1).toInt());
      }
    }
    line = in.readLine();
  }
  file.close();

  if (strList.count() == 1) {
    int64_t endTime = strList.at(0).toLongLong();

    if (endTime > tmp) {
      tmp = endTime;
    }
    ui->timeLineWidget->setTimeEnd(tmp);
  }

  ui->actionFindTimePoint->setEnabled(true);
  ui->actionFindTimeLength->setEnabled(true);
  ui->timeGridPushButton->setEnabled(true);
  ui->timeLineWidget->adjustTimeGrid();
}

void MainWindow::showHelpMenuInfo(void) {
  QObject* obj = QObject::sender();

  if (obj == ui->actionAbout) {
    _msgDialog->setWindowTitle(tr("About"));
    _msgDialog->setMessage(aboutMessage);
    goto showMsgLabel;
  }

  if (obj == ui->actionLicense) {
    _msgDialog->setWindowTitle(tr("License"));
    _msgDialog->setMessage(licenseMessage);

  showMsgLabel:
    _msgDialog->adjustSize();
    _msgDialog->show();
  }
}
