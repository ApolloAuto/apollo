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

#ifndef TOOLS_CVT_VTOPOLOGY_INTERACTIVEGRAPHICSVIEW_H_
#define TOOLS_CVT_VTOPOLOGY_INTERACTIVEGRAPHICSVIEW_H_

#include <QGraphicsView>

class InteractiveGraphicsView : public QGraphicsView {
  Q_OBJECT

 public:
  explicit InteractiveGraphicsView(QWidget *parent = 0);
  explicit InteractiveGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);
  ~InteractiveGraphicsView() {}

  void zoom_in();
  void zoom_out();
  void zoom(float scaleValue);
  void translate(QPointF deltaVal);

 protected:
  void keyPressEvent(QKeyEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

 private:
  void init(void);

  bool mouse_left_button_pressed_;
  QPoint last_mouse_pos_;
  double scaleVal_;
};

#endif  // INTERACTIVEGRAPHICSVIEW_H
