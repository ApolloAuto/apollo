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

#include "time_line_row.h"
#include <QGraphicsScene>
#include <iostream>
#include "time_block_item.h"
#include "time_line_scene.h"

namespace {
QList<QColor> _colorsList{

    QRgb(0x209fdf), QRgb(0x99ef53), QRgb(0xf6a625),
    QRgb(0x6defd5), QRgb(0xbf593e),

    QRgb(0xdf209f), QRgb(0x5399ef), QRgb(0x25f6a6),
    QRgb(0xd56def), QRgb(0x3ebf59),

    QRgb(0x9fdf20), QRgb(0xef5399), QRgb(0xa625f6),
    QRgb(0xefd56d), QRgb(0x593ebf),
};

#define getNewColor(i) _colorsList.at(i)
}

TimeLineRow::TimeLineRow(QGraphicsItem* parent, const QString& rowName)
    : QGraphicsItemGroup(parent),
      _colorIndex(0),
      _itemsMap(),
      _headerText(rowName),
      _backgroud() {
  setPos(0, 0);
  addToGroup(&_headerText);
  addToGroup(&_backgroud);

  _headerText.setPos(0, 0);
  _backgroud.setPos(0, 0);

  _headerText.setVisible(false);
  _headerText.setToolTip(rowName);

  _backgroud.setZValue(-1);

  QBrush b(Qt::gray);
  b.setStyle(Qt::SolidPattern);
  _backgroud.setBrush(b);
  _backgroud.setPen(QPen(Qt::gray));

  setFocusProxy(&_backgroud);
}

TimeLineRow::~TimeLineRow() {
  removeFromGroup(&_headerText);
  removeFromGroup(&_backgroud);

  QGraphicsScene* s = scene();
  s->removeItem(&_headerText);
  s->removeItem(&_backgroud);

  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
    QList<TimeBlockItem*>* list = iter.value();
    for (TimeBlockItem* item : *list) {
      delete item;
    }
    list->clear();
    delete list;
  }
  _itemsMap.clear();
}

QRectF TimeLineRow::boundingRect() const { return _backgroud.boundingRect(); }

void TimeLineRow::setRowHeaderText(const QString& rowName) {
  if (!_headerText.isVisible()) _headerText.setVisible(true);

  _headerText.setPlainText(rowName);
  _headerText.setToolTip(rowName);
}

TimeBlockItem* TimeLineRow::addTimeBlock(const QString& timeBlockName,
                                         const int64_t& start,
                                         const int64_t& end,
                                         bool isEventBlock) {
  TimeBlockItem* item = new TimeBlockItem(start, end);
  if (item == nullptr) {
    return nullptr;
  }

  item->setToolTip(timeBlockName);
  addToGroup(item);
  item->setPos(0, 0);

  QList<TimeBlockItem*>* list = nullptr;
  QMap<QString, QList<TimeBlockItem*>*>::iterator iter =
      _itemsMap.find(timeBlockName);

  if (iter == _itemsMap.end()) {
    list = new QList<TimeBlockItem*>();
    if (list == nullptr) {
      delete item;
      return nullptr;
    }
    _itemsMap.insert(timeBlockName, list);

    if (!isEventBlock) {
      item->setColor(getNewColor(_colorIndex));
      _colorIndex = (_colorIndex + 1) % _colorsList.size();
    }
  } else {
    list = iter.value();
    if (!isEventBlock) {
      item->setColor(list->at(0)->itemColor());
    }
  }

  list->append(item);

  return item;
}

void TimeLineRow::setHeaderWidth(qreal offsetX) {
  _headerText.moveBy(offsetX / 2.0, 0);
  moveTimeBlockLr(offsetX);
}

void TimeLineRow::setHeight(qreal h) {
  QRectF r = _backgroud.rect();
  if (r.height() == h) return;

  r.setHeight(h);
  _backgroud.setRect(r);

  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
    QList<TimeBlockItem*>* list = iter.value();
    for (TimeBlockItem* item : *list) {
      r = item->rect();
      r.setHeight(h);
      item->setRect(r);
    }
  }
}

void TimeLineRow::moveTimeBlockLr(qreal offsetX) {
  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
    QList<TimeBlockItem*>* list = iter.value();
    for (TimeBlockItem* item : *list) {
      item->setX(item->x() + offsetX);
    }
  }
}

void TimeLineRow::recalculateTimeBlock(int64_t& base, int timeRatio, int scale,
                                       qreal rowHeaderWidth) {
  int64_t start, end;

  qreal h = _backgroud.rect().height();

  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
    QList<TimeBlockItem*>* list = iter.value();
    for (TimeBlockItem* item : *list) {
      start = item->startTimeStamp();
      end = item->endTimeStamp();

      bool isSpecialItem = false;
      if (start < 1LL) {
        start = end;
        isSpecialItem = true;
      }

      end -= start;
      start -= base;

      end *= scale;
      start *= scale;

      start /= timeRatio;
      end /= timeRatio;

      if(isSpecialItem){
          end = 1;
      }

      item->setRect(0, 0, end, h);
      item->setPos(start + rowHeaderWidth, 0);
    }
  }
}

void TimeLineRow::clear(void) {
  _headerText.setToolTip(QString(""));
  _headerText.setPlainText(QString(""));

  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
    QList<TimeBlockItem*>* list = iter.value();
    for (TimeBlockItem* item : *list) {
      delete item;
    }
    list->clear();

    delete list;
  }
  _itemsMap.clear();
}

#undef getNewColor
