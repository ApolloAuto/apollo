/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "find_widget.h"
#include <QRegExp>
#include <memory>
#include "ui_find_timelength.h"
#include "ui_find_timepoint.h"
#include "ui_find_widget.h"

namespace {

bool (*__funcs__[])(double v, double s) = {
    [](double v, double s) -> bool {  // greater than
      if (v > s) {
        return true;
      } else {
        return false;
      }
    },
    [](double v, double s) -> bool {  // less than
      if (v < s) {
        return true;
      } else {
        return false;
      }
    },
    [](double v, double s) -> bool {  // equal
      if (v == s) {
        return true;
      } else {
        return false;
      }
    },
    [](double v, double s) -> bool {  // equal or greater than
      if (v >= s) {
        return true;
      } else {
        return false;
      }
    },
    [](double v, double s) -> bool {  // equal or less than
      if (v <= s) {
        return true;
      } else {
        return false;
      }
    }};
}

FindWidget::FindWidget(QWidget* parent)
    : QWidget(parent),
      ui(new Ui::FindWidget),
      timeLengthUi(new Ui::FindTimeLength),
      timePointUi(new Ui::FindTimePoint),
      _currentFindMode(FindTimeLengthMode) {
  ui->setupUi(this);

  timeLengthUi->setupUi(ui->contentWidget);
  memset(timePointUi, 0, sizeof(Ui::FindTimePoint));

  connect(ui->toolButton, SIGNAL(clicked(bool)), this, SLOT(privateHide()));
}

FindWidget::~FindWidget() {
  delete ui;
  delete timeLengthUi;
  delete timePointUi;
}

void FindWidget::privateHide(void) {
  hide();
  emit isHiding();
}

int FindWidget::selectedProcessor(void) const {
  return timeLengthUi->procComboBox->currentIndex();
}

void FindWidget::setProcessorCount(int c) {
  if (_currentFindMode == FindMode::FindTimeLengthMode) {
    timeLengthUi->procComboBox->clear();
    for (int i = 0; i < c; ++i) {
      timeLengthUi->procComboBox->addItem(tr("%1").arg(i));
    }
  } else {
    timePointUi->procComboBox->clear();
    for (int i = 0; i < c; ++i) {
      timePointUi->procComboBox->addItem(tr("%1").arg(i));
    }
  }
}

bool FindWidget::isFound(double v) {
  if (_currentFindMode == FindTimeLengthMode) {
    return ::__funcs__[timeLengthUi->operatorComboBox->currentIndex()](
        v, timeLengthUi->timeSpinBox->value());
  } else {
    bool isOk;
    double d = timePointUi->timePointLineEdit->text().toDouble(&isOk);
    if (isOk) {
      isOk = __funcs__[2](v, d);
    }
    return isOk;
  }
}

void FindWidget::switchFindMode(FindMode m) {
  if (_currentFindMode == m) return;
  _currentFindMode = m;

  ui->contentWidget->setVisible(false);

  const QObjectList& childWidgets = ui->contentWidget->children();
  for (auto item : childWidgets) {
    delete item;
  }

  switch (m) {
    case FindTimeLengthMode:
      timeLengthUi->setupUi(ui->contentWidget);
      break;
    case FindTimePointMode:
      timePointUi->setupUi(ui->contentWidget);
      QRegExpValidator* dv = new QRegExpValidator(QRegExp("\\d+"));
      if (dv) timePointUi->timePointLineEdit->setValidator(dv);
  }

  adjustSize();

  ui->contentWidget->setVisible(true);
}
