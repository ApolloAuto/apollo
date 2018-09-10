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

#ifndef TIME_LINE_WIDGET_H
#define TIME_LINE_WIDGET_H

#include <QWidget>
#include "time_line_scene.h"

namespace Ui {
class TimeLineWidget;
}

class QGraphicsItem;

class TimeLineWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TimeLineWidget(QWidget* parent = 0);
  ~TimeLineWidget();

  int64_t timeBegin(void) const { return _scene->timeBegin(); }
  int64_t timeEnd(void) const { return _scene->timeEnd(); }

  void setTimeBegin(int64_t b) { _scene->setTimeBegin(b); }
  void setTimeEnd(int64_t t) { _scene->setTimeEnd(t); }

  int timeLineLength(void) const { return _scene->timeLineLength(); }

  int timeRatio(void) const { return _scene->timeRatio(); }

  int axisScale(void) const { return _scene->axisScale(); }
  int axisSparsity(void) const { return _scene->axisSparsity(); }
  int rowHeaderWidth(void) const { return _scene->rowHeaderWidth(); }
  int rowHeight(void) const { return _scene->rowHeight(); }

  int sceneWidth(void) const { return _scene->sceneWidth(); }

  TimeBlockItem* addTimeBlock(int processorIndex, const QString& blockName,
                              int64_t& start, int64_t& end) {
    return _scene->addTimeBlock(processorIndex, blockName, start, end);
  }

  void adjustTimeGrid(void) { _scene->adjustTimeGrid(); }

 public slots:
  void setTimeRatio(int tr);
  void setSceneScale(int s);
  void setSceneSparsity(int s);
  void setRowHeaderWidth(int w);
  void setRowHeight(int h);
  void lrScroll(int);

  void showFindWindow(void);
  void showFindTimePointWindow(void);
  void showTimeGrid(bool b);

 protected:
  void resizeEvent(QResizeEvent* e);
  void wheelEvent(QWheelEvent* e);

 private slots:
  void viewDoubleClicked(QGraphicsItem*, QPointF& scenePos);
  void findTimeLength(void);
  void clearFindResults(void);

 private:
  void refindTimeLength(void);

  Ui::TimeLineWidget* ui;
  TimeLineScene* _scene;

  int _resultIndex;
  int _currentIndex;
  QList<TimeBlockItem*> _findResults;
};

#endif  // TIME_LINE_WIDGET_H
