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
#include "has_data_come_event.h"
#include "loader_thread.h"
#include "msg_dialog.h"
#include "perf_data_base.h"
#include "time_block_item.h"
#include "time_block_item_pool.h"
#include "time_line_scene.h"
#include "time_line_widget.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QWheelEvent>
#include <iostream>

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
      _msgDialog(new MessageDialog),
      _loaderThread(new LoaderThread(this)),
      _maxDataBlockIndex(-1) {
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
  connect(ui->actionSelectEventId, SIGNAL(triggered(bool)), ui->timeLineWidget,
          SLOT(showFindEventIdWindow()));

  connect(ui->timeGridPushButton, SIGNAL(clicked(bool)), this,
          SLOT(showTimeGrid(bool)));

  connect(_loaderThread, SIGNAL(finished()), this, SLOT(handleSparerResult()));
}

MainWindow::~MainWindow() {
  if (_loaderThread->isRunning()) {
    _loaderThread->stop();
    _loaderThread->quit();
  }

  if (!_loaderThread->isFinished()) {
    _loaderThread->wait();
  }

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

  if (_maxDataBlockIndex > -1) {
    disconnect(ui->dataBlockSpinBox, SIGNAL(valueChanged(int)), this,
               SLOT(showDataBlock(int)));
  }

  ui->timeLineWidget->resetAll();

  _maxDataBlockIndex = -1;
  ui->actionFindTimePoint->setEnabled(false);
  ui->actionFindTimeLength->setEnabled(false);
  ui->timeGridPushButton->setEnabled(false);
  ui->dataBlockSpinBox->setEnabled(false);
  ui->dataBlockSpinBox->setValue(0);

  showTimeGrid(false);
  ui->timeGridPushButton->setChecked(false);

  ui->statusBar->showMessage(tr("Parsing Perf Data..."));

  _loaderThread->setPerfDataFile(fileName);
  _loaderThread->start();
}
void MainWindow::handleSparerResult() {
  ui->statusBar->showMessage(
      tr("Parse Over! %1 data blocks").arg(_maxDataBlockIndex + 1));
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

void MainWindow::showDataBlock(int index) {
  PerfDatabase* db = PerfDatabase::instance();
  PerfBlockDatabase* block = db->getSubDataBase(index);
  ui->timeLineWidget->drawBlockDatabase(block);
}

bool MainWindow::event(QEvent* e) {
  if (e->type() == HasDataComeEvent::staticType()) {
    HasDataComeEvent* dataComeEvent = static_cast<HasDataComeEvent*>(e);
    int v = _maxDataBlockIndex;
    int newIndex = dataComeEvent->dataBlockIndex();

    if (_maxDataBlockIndex < newIndex) {
      _maxDataBlockIndex = newIndex;
      ui->dataBlockSpinBox->setMaximum(newIndex);
    }
    if (v < 0) {
      PerfDatabase* db = PerfDatabase::instance();
      PerfBlockDatabase* block = db->getSubDataBase(0);
      TimeBlockItemPool::instance()->extend(block->timeBlockCount());

      ui->timeLineWidget->drawBlockDatabase(block);

      QString&& s = tr("%1").arg(db->startTimeStamp());
      ui->timeBaseContentLabel->setText(s);
      ui->timeBaseContentLabel->setToolTip(s);

      ui->actionFindTimePoint->setEnabled(true);
      ui->actionFindTimeLength->setEnabled(true);
      ui->timeGridPushButton->setEnabled(true);
      ui->dataBlockSpinBox->setEnabled(true);
      ui->timeLineWidget->adjustTimeGrid();

      connect(ui->dataBlockSpinBox, SIGNAL(valueChanged(int)), this,
              SLOT(showDataBlock(int)));
    }

    return true;
  }

  return QMainWindow::event(e);
}
