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

#include "time_line_legend_item.h"
#include <QGraphicsScene>

TimeLineLegendItem::TimeLineLegendItem(QGraphicsItem *parent)
    : QGraphicsItemGroup(parent), _legendColor(), _legendText() {
  setPos(0, 0);
  addToGroup(&_legendColor);
  addToGroup(&_legendText);

  _legendColor.setRect(0, 0, 5, 5);
  _legendColor.setPos(0, 0);
  _legendText.setPos(8, -3.0f);
}

TimeLineLegendItem::~TimeLineLegendItem() {
  removeFromGroup(&_legendColor);
  removeFromGroup(&_legendText);

  QGraphicsScene *s = scene();
  s->removeItem(&_legendColor);
  s->removeItem(&_legendText);
}

QRectF TimeLineLegendItem::boundingRect() const {
  QRectF r = _legendColor.boundingRect();
  QRectF t = _legendText.boundingRect();
  qreal tmp = r.height();
  if (tmp < t.height()) tmp = t.height();
  r.setHeight(tmp);
  tmp = r.width();
  tmp += t.width();
  tmp += 3.0f;
  r.setWidth(tmp);
  return r;
}
