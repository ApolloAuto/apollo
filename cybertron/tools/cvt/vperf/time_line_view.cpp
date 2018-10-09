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

#include "time_line_view.h"
#include <QGraphicsItem>
#include <QMouseEvent>
#include <iostream>

TimeLineView::TimeLineView(QWidget *parent)
    : QGraphicsView(parent), _lastY(0), _targetHeight(0) {
  _targetHeight = height();
}

TimeLineView::TimeLineView(QGraphicsScene *scene, QWidget *parent)
    : QGraphicsView(scene, parent), _lastY(0.0f), _targetHeight(0) {
  _targetHeight = height();
}

void TimeLineView::wheelEvent(QWheelEvent *e) { QFrame::wheelEvent(e); }

void TimeLineView::mouseDoubleClickEvent(QMouseEvent *e) {
  QGraphicsView::mouseDoubleClickEvent(e);
  QPoint p = e->pos();
  QGraphicsItem *item = itemAt(p);
  QPointF scenePos = mapToScene(p);
  emit viewDoubleClicked(item, scenePos);
}

void TimeLineView::mousePressEvent(QMouseEvent *e) {
  if (e->buttons() == Qt::LeftButton) {
    _lastY = e->pos().y();
  }

  QGraphicsView::mousePressEvent(e);
}

void TimeLineView::mouseMoveEvent(QMouseEvent *e) {
  if (e->buttons() == Qt::LeftButton) {
    QRectF r = sceneRect();
    qreal y = r.y() + _lastY - e->pos().y();
    _lastY = e->pos().y();

    if (y < 0) y = 0.0f;
    if (y + r.height() < _targetHeight) {
      setSceneRect(r.x(), y, width() - 2, height() - 2);
    }
  }

  QGraphicsView::mouseMoveEvent(e);
}
