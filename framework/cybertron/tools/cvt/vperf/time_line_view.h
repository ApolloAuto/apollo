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

#ifndef TOOLS_CVT_VPERF_TIMELINEVIEW_H_
#define TOOLS_CVT_VPERF_TIMELINEVIEW_H_

#include <QGraphicsView>

class TimeBlockItem;
class QGraphicsItem;

class TimeLineView : public QGraphicsView {
  Q_OBJECT
 public:
  explicit TimeLineView(QWidget *parent = nullptr);
  explicit TimeLineView(QGraphicsScene *scene, QWidget *parent = nullptr);

  void setTargetHeight(int h) { _targetHeight = h; }
  int targetHeight(void) const { return _targetHeight; }

 signals:
  void viewDoubleClicked(QGraphicsItem *item, QPointF &scenePos);

 protected:
  void wheelEvent(QWheelEvent *e);
  void mouseDoubleClickEvent(QMouseEvent *e);

  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);

 private:
  int _lastY;
  int _targetHeight;
};

#endif  // TIMELINEVIEW_H
