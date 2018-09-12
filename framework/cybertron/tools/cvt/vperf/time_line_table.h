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

#ifndef TIMELINETABLE_H
#define TIMELINETABLE_H

#include <QGraphicsItemGroup>
#include <QGraphicsRectItem>
#include <QList>
#include "perf_data_base.h"

class TimeLineRow;

class TimeLineTable : public QGraphicsItemGroup {
 public:
  enum { Type = QGraphicsItemGroup::UserType + QGraphicsItemGroup::Type + 0 };

  explicit TimeLineTable(QGraphicsItem *parent = nullptr);
  ~TimeLineTable();

  int type() const { return Type; }
  QRectF boundingRect() const override;

  void setWidth(qreal w);
  void setRowOffset(qreal offset);
  void setRowHeight(qreal rowHeight);
  void setRowHeaderWidth(qreal rowHeaderWidth);
  bool setRowCount(int rowCount);

  qreal width(void) const { return _width; }
  qreal height(void) const {
    qreal h = _rowList.count() * (_rowHeight + _rowOffset);
    h -= _rowOffset;
    if (h < 0) h = 0.0f;
    return h;
  }

  qreal rowOffset(void) const { return _rowOffset; }
  qreal rowHeight(void) const { return _rowHeight; }
  qreal rowHeaderWidth(void) const { return _columnLine.x(); }
  int rowCount(void) const { return _rowList.count(); }
  void append(TimeLineRow *row);

  TimeLineRow *rowAt(int index) {
    if (index > -1 && index < _rowList.count()) {
      return _rowList.at(index);
    } else {
      return nullptr;
    }
  }

  void recalculateTimeBlock(int64_t &base, int timeRatio, int scale);

  void destroyAllRows(void);

 private:
  void updateHeightOffset(void);

  qreal _rowHeight;
  qreal _width;
  qreal _rowOffset;

  QGraphicsLineItem _columnLine;
  QList<TimeLineRow *> _rowList;
};

#endif  // TIMELINETABLE_H
