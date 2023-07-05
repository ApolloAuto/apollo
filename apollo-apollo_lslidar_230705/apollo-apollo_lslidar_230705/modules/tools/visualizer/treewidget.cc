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

#include "modules/tools/visualizer/treewidget.h"

#include <QtGui/QResizeEvent>

TreeWidget::TreeWidget(QWidget *parent) : QTreeWidget(parent) {}

void TreeWidget::resizeEvent(QResizeEvent *event) {
  QTreeWidget::resizeEvent(event);
  int cw = width() / columnCount();
  for (int i = 0; i < columnCount(); ++i) {
    setColumnWidth(i, cw);
  }
}

bool TreeWidget::event(QEvent *e) {
  bool b = QTreeWidget::event(e);
  if (e->type() == QEvent::Hide) {
    emit visibilityChanged(false);
  }
  if (e->type() == QEvent::Show) {
    emit visibilityChanged(true);
  }
  return b;
}
