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

#ifndef TOOLS_CVT_VPERF_TIMELINELEGEND_H_
#define TOOLS_CVT_VPERF_TIMELINELEGEND_H_

#include <QGraphicsItemGroup>
#include <QList>

class TimeLineLegendItem;

class TimeLineLegend : public QGraphicsItemGroup {
 public:
  enum { Type = QGraphicsItemGroup::UserType + QGraphicsItemGroup::Type + 3 };

  explicit TimeLineLegend(QGraphicsItem *parent = nullptr);
  ~TimeLineLegend();

  int type() const { return Type; }

  bool setLegendItemsCount(int count);

  TimeLineLegendItem *itemByIndex(int index) {
    if (index > -1 && index < _items.count()) {
      return _items.at(index);
    }
    return nullptr;
  }

  qreal legendWidth(void);

  void adjustItemsPos(void);

 private:
  QList<TimeLineLegendItem *> _items;
};

#endif  // TIMELINELEGEND_H
