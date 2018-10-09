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

#include "time_line_legend.h"
#include "time_line_legend_item.h"

TimeLineLegend::TimeLineLegend(QGraphicsItem *parent)
    : QGraphicsItemGroup(parent), _items() {}

TimeLineLegend::~TimeLineLegend() {
  for (auto item : _items) {
    delete item;
  }
  _items.clear();
}

qreal TimeLineLegend::legendWidth() {
  qreal w = 0.0f;
  for (auto item : _items) {
    w += item->itemWidth();
    w += 3.0f;
  }

  w -= 3.0f;
  if (w < 0.0f) w = 0.0f;

  return w;
}

bool TimeLineLegend::setLegendItemsCount(int count) {
  int c = _items.count();
  if (count > c) {
    qreal w = legendWidth();
    if (w > 0.0f) w += 3.0f;
    int i = c;
    for (; i < count; ++i) {
      TimeLineLegendItem *item = new TimeLineLegendItem(this);
      if (item) {
        addToGroup(item);
        item->setPos(w, 0);
        w += item->itemWidth() + 3.0f;

        _items.append(item);
      } else {
        break;
      }
    }

    if (i < count) {
      for (; c < i; ++c) {
        TimeLineLegendItem *item = _items.takeLast();
        delete item;
      }

      return false;
    }
  } else {
    for (; count < c; ++count) {
      TimeLineLegendItem *item = _items.takeLast();
      delete item;
    }
  }

  return true;
}

void TimeLineLegend::adjustItemsPos(void) {
  qreal w = 0.0f;
  for (auto item : _items) {
    item->setPos(w, 0);
    w += item->itemWidth() + 3.0f;
  }
}
