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

#include "time_line_table.h"
#include <QGraphicsScene>
#include <cassert>
#include <iostream>
#include "time_line_row.h"

TimeLineTable::TimeLineTable(QGraphicsItem *parent)
    : QGraphicsItemGroup(parent),
      _rowHeight(40.0f),
      _width(300.0f),
      _rowOffset(5.0f),
      _columnLine(),
      _rowList() {
  setPos(0, 0);
  addToGroup(&_columnLine);

  _columnLine.setPos(100, 0);
  _columnLine.setZValue(1);
}

TimeLineTable::~TimeLineTable() {
  removeFromGroup(&_columnLine);
  _rowList.clear();
}

QRectF TimeLineTable::boundingRect() const {
  QRectF r;
  r.setWidth(_width);
  r.setHeight(height());

  return r;
}

bool TimeLineTable::setRowCount(int rowCount) {
  int c = _rowList.count();
  if (rowCount == c) return true;
  if (rowCount > c) {  // add
    int i = c;
    for (; i < rowCount; ++i) {
      TimeLineRow *r = new TimeLineRow(this);
      if (r) {
        addToGroup(r);
        r->setPos(0, i * (_rowHeight + _rowOffset));
        r->setHeight(_rowHeight);
        r->setWidth(_width);
        _rowList.append(r);
      } else {
        break;
      }
    }

    if (i != rowCount) {
      for (; c < i; ++c) {
        TimeLineRow *item = _rowList.takeLast();
        delete item;
      }
      return false;
    }

  } else {
    for (int i = rowCount; i < c; ++i) {
      TimeLineRow *item = _rowList.takeLast();
      delete item;
    }
  }

  QPointF p = _columnLine.pos();
  qreal y = rowCount * (_rowHeight + _rowOffset);
  y -= _rowOffset;
  if (y < 0) y = 0.0f;
  _columnLine.setLine(0, 0, 0, y);
  _columnLine.setPos(p);
  return true;
}

void TimeLineTable::append(TimeLineRow *row) {
  assert(row != nullptr);

  addToGroup(row);

  int c = _rowList.count();

  qreal y = c * (_rowHeight + _rowOffset);
  y -= _rowOffset;
  if (y < 0) y = 0.0f;

  row->setPos(0, y);

  row->setHeight(_rowHeight);
  row->setWidth(_width);

  QPointF p = _columnLine.pos();
  _columnLine.setLine(0, 0, 0, y + _rowHeight);
  _columnLine.setPos(p);
  _rowList.append(row);
}

void TimeLineTable::setRowOffset(qreal offset) {
  _rowOffset = offset;
  updateHeightOffset();
}

void TimeLineTable::setRowHeight(qreal rowHeight) {
  _rowHeight = rowHeight;
  updateHeightOffset();
}

void TimeLineTable::updateHeightOffset(void) {
  qreal y = 0.0f;
  for (auto item : _rowList) {
    item->setY(y);
    item->setHeight(_rowHeight);

    y += _rowHeight;
    y += _rowOffset;
  }

  y -= _rowOffset;
  if (y < 0) y = 0;

  QPointF p = _columnLine.pos();
  _columnLine.setLine(0, 0, 0, y);
  _columnLine.setPos(p);
}

void TimeLineTable::setWidth(qreal w) {
  _width = w;
  for (auto item : _rowList) {
    item->setWidth(w);
  }
}

void TimeLineTable::setRowHeaderWidth(qreal rhw) {
  qreal delta = rhw - rowHeaderWidth();
  if (delta == 0.0f) return;

  for (auto item : _rowList) {
    item->setHeaderWidth(delta);
  }

  _columnLine.setPos(rhw, 0);
}

void TimeLineTable::recalculateTimeBlock(int64_t &base, int timeRatio,
                                         int scale) {
  for (auto item : _rowList) {
    item->recalculateTimeBlock(base, timeRatio, scale, rowHeaderWidth());
  }
}

void TimeLineTable::destroyAllRows(void) {
  for (auto item : _rowList) {
    delete item;
  }
  _rowList.clear();
}
