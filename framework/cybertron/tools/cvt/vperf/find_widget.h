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

#ifndef TOOLS_CVT_VPERF_FIND_WIDGET_H_
#define TOOLS_CVT_VPERF_FIND_WIDGET_H_

#include <QWidget>

namespace Ui {
class FindWidget;
class FindTimeLength;
class FindTimePoint;
class FindEventIdWidget;
}

class TimeLineWidget;

class FindWidget : public QWidget {
  Q_OBJECT

 public:
  enum FindMode { FindTimeLengthMode, FindTimePointMode, FindEvnetId };

  explicit FindWidget(QWidget *parent = 0);
  ~FindWidget();

  int selectedProcessor(void) const;
  void setProcessorCount(int c);

  bool isFound(double v);

  void switchFindMode(FindMode m);
  FindMode findMode(void) const { return _currentFindMode; }

 signals:
  void isHiding(void);

 private slots:
  void privateHide(void);

 private:
  Ui::FindWidget *ui;
  QWidget *shadowWidget;
  Ui::FindTimeLength *timeLengthUi;
  Ui::FindTimePoint *timePointUi;
  Ui::FindEventIdWidget *eventIdUi;

  FindMode _currentFindMode;

  friend class TimeLineWidget;
};

#endif  // TOOLS_CVT_VPERF_FIND_WIDGET_H_
