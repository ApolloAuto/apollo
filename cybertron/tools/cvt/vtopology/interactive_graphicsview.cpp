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

#include "./interactive_graphicsview.h"

#include <iostream>

#include <QKeyEvent>
#include <QWheelEvent>

namespace {
constexpr double translate_speed_ = 1.0;
constexpr double zoom_delta_ = 0.1;
}

InteractiveGraphicsView::InteractiveGraphicsView(QWidget *parent)
    : QGraphicsView(parent) {
  init();
}

InteractiveGraphicsView::InteractiveGraphicsView(QGraphicsScene *scene,
                                                 QWidget *parent)
    : QGraphicsView(scene, parent) {
  init();
}

void InteractiveGraphicsView::init(void) {
  mouse_left_button_pressed_ = false;
  scaleVal_ = 1.0;

  setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setCursor(Qt::PointingHandCursor);
  setRenderHint(QPainter::Antialiasing);

  setSceneRect(INT_MIN, INT_MIN, INT_MAX, INT_MAX);
  centerOn(0, 0);
}

void InteractiveGraphicsView::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Up:
      translate(QPointF(0, -2));
      goto acceptLabel;
    case Qt::Key_Down:
      translate(QPointF(0, 2));
      goto acceptLabel;
    case Qt::Key_Left:
      translate(QPointF(-2, 0));
      goto acceptLabel;
    case Qt::Key_Right:
      translate(QPointF(2, 0));
      goto acceptLabel;
    case Qt::Key_Plus:
      zoom_in();
      goto acceptLabel;
    case Qt::Key_Minus:
      zoom_out();

    acceptLabel:
      event->setAccepted(true);
    default:
      QGraphicsView::keyPressEvent(event);
  }
}

void InteractiveGraphicsView::mouseMoveEvent(QMouseEvent *event) {
  if (mouse_left_button_pressed_) {
    QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mouse_pos_);
    translate(mouseDelta);
  }
  last_mouse_pos_ = event->pos();

  QGraphicsView::mouseMoveEvent(event);
}

void InteractiveGraphicsView::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    QPointF point = mapToScene(event->pos());
    if (scene()->itemAt(point, transform()) == nullptr) {
      last_mouse_pos_ = event->pos();

      mouse_left_button_pressed_ = true;
    }
  }

  QGraphicsView::mousePressEvent(event);
}

void InteractiveGraphicsView::mouseReleaseEvent(QMouseEvent *event) {
  mouse_left_button_pressed_ = false;
  QGraphicsView::mouseReleaseEvent(event);
}

void InteractiveGraphicsView::wheelEvent(QWheelEvent *event) {
  QPoint scrollAmount = event->angleDelta();
  scrollAmount.y() > 0 ? zoom_in() : zoom_out();
}

void InteractiveGraphicsView::zoom_in() { zoom(1 + zoom_delta_); }

void InteractiveGraphicsView::zoom_out() { zoom(1 - zoom_delta_); }

void InteractiveGraphicsView::zoom(float scalValue) {
  double factor = transform()
                      .scale(scalValue, scalValue)
                      .mapRect(QRectF(0, 0, 1, 1))
                      .width();
  if (factor < 0.07 || factor > 100) return;

  scale(scalValue, scalValue);
  scaleVal_ *= scalValue;
}

void InteractiveGraphicsView::translate(QPointF delta) {
  delta *= scaleVal_;
  delta *= translate_speed_;

  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  QPoint newCenter(viewport()->rect().width() / 2 - delta.x(),
                   viewport()->rect().height() / 2 - delta.y());
  centerOn(mapToScene(newCenter));

  setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}
