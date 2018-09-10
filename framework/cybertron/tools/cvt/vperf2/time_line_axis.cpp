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

#include "time_line_axis.h"
#include <QGraphicsLineItem>
#include <iostream>
#include "time_line_scene.h"

TimeLineAxis::TimeLineAxis(QGraphicsItem* parent)
    : QGraphicsItemGroup(parent), _headerText() {
  addToGroup(&_headerText);

  _headerText.setPos(0, 0);
  _headerText.setZValue(1);
  _headerText.setVisible(false);
}

TimeLineAxis::~TimeLineAxis() {
  removeFromGroup(&_headerText);
  scene()->removeItem(&_headerText);
}

void TimeLineAxis::setHeaderText(const QString& rowName) {
  if (!_headerText.isVisible()) _headerText.setVisible(true);

  _headerText.setPlainText(rowName);
  _headerText.setToolTip(rowName);
}

void TimeLineAxis::destroy(void) {
  _headerText.setVisible(false);
  removeFromGroup(&_headerText);

  QList<QGraphicsItem*> items = childItems();
  for (auto item : items) {
    delete item;
  }

  addToGroup(&_headerText);
  _headerText.setVisible(true);

  items = childItems();
}

void TimeLineAxis::draw(int headerWidth, int timeLineLength, int height,
                        int axisScale, int axisSparsity) {
  QPen p(Qt::black);

  int px = x();
  int py = y();

  QGraphicsLineItem* item = new QGraphicsLineItem();  // top line
  if (!item) {
    return;
  }
  addToGroup(item);
  item->setPen(p);
  item->setLine(px, py, px + headerWidth + timeLineLength * axisScale, py);

  item = new QGraphicsLineItem();  // bottom line
  if (!item) {
    return;
  }
  addToGroup(item);
  item->setPen(p);
  item->setLine(px, py + height, px + headerWidth + timeLineLength * axisScale,
                py + height);

  px += headerWidth;

  item = new QGraphicsLineItem();  // 0 line
  if (!item) {
    return;
  }
  addToGroup(item);
  item->setPen(p);
  item->setLine(px, py, px, py + height);

  int c = 0;
  int h0 = height;
  int h1 = height << 1;
  int h2 = h1 + height;
  h0 /= 5;
  h1 /= 5;
  h2 /= 4;

  int offsetX = headerWidth;

  QGraphicsTextItem* textItem = new QGraphicsTextItem(QObject::tr("%1").arg(c));
  if (!textItem) {
    return;
  }
  addToGroup(textItem);
  textItem->setPos(offsetX - 3, h2 - 15);

  int sparsity2 = axisSparsity << 1;
  px += axisScale;
  offsetX += axisScale;
  ++c;

  for (; c < timeLineLength; px += axisScale, offsetX += axisScale, ++c) {
    int y;
    bool isMaxLine;

    y = h0;
    isMaxLine = false;

    if (c % sparsity2 == 0) {
      y = h2;
      isMaxLine = true;
    } else if (c % axisSparsity == 0) {
      y = h1;
    }

    item = new QGraphicsLineItem();
    if (item == nullptr) break;
    addToGroup(item);
    item->setPen(p);
    item->setLine(px, py, px, py + y);

    if (isMaxLine) {
      textItem = new QGraphicsTextItem(QObject::tr("%1").arg(c));
      if (textItem) {
        addToGroup(textItem);
        textItem->setPos(offsetX - 3, h2 - 15);
      }
    }
  }
}
