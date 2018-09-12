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
#include "time_block_item_pool.h"
#include "time_line_scene.h"

TimeLineRow::TimeLineRow(QGraphicsItem *parent, const QString &rowName)
    : QGraphicsItemGroup(parent), _headerText(rowName), _backgroud() {
  setPos(0, 0);
  addToGroup(&_headerText);
  addToGroup(&_backgroud);

  _headerText.setPos(0, 0);
  _backgroud.setPos(0, 0);

  _headerText.setVisible(false);
  _headerText.setToolTip(rowName);

  _backgroud.setZValue(-1);

  QColor c(Qt::gray);
  QBrush b(c);
  b.setStyle(Qt::SolidPattern);
  _backgroud.setBrush(b);
  _backgroud.setPen(QPen(c));

  setFocusProxy(&_backgroud);
}

TimeLineRow::~TimeLineRow() {
  removeFromGroup(&_headerText);
  removeFromGroup(&_backgroud);

  QGraphicsScene *s = scene();
  s->removeItem(&_headerText);
  s->removeItem(&_backgroud);

  TimeBlockItemPool *p = TimeBlockItemPool::instance();

  QList<QGraphicsItem *> children = childItems();
  for (auto item : children) {
    removeFromGroup(item);
    s->removeItem(item);

    p->deallocate(static_cast<TimeBlockItem *>(item));
  }
}

QRectF TimeLineRow::boundingRect() const { return _backgroud.boundingRect(); }

void TimeLineRow::setRowHeaderText(const QString &n) {
  if (!_headerText.isVisible()) _headerText.setVisible(true);

  _headerText.setPlainText(n);
  _headerText.setToolTip(n);
}

// TimeBlockItem *TimeLineRow::addTimeBlock(const QString &timeBlockName,
//                                         const int64_t &start,
//                                         const int64_t &end,
//                                         bool isEventBlock) {
//  TimeBlockItem *item = new TimeBlockItem(start, end);
//  if (item == nullptr) {
//    return nullptr;
//  }

//  item->setToolTip(timeBlockName);
//  addToGroup(item);
//  item->setPos(0, 0);

//  QList<TimeBlockItem *> *list = nullptr;
//  QMap<QString, QList<TimeBlockItem *> *>::iterator iter =
//      _itemsMap.find(timeBlockName);

//  if (iter == _itemsMap.end()) {
//    list = new QList<TimeBlockItem *>();
//    if (list == nullptr) {
//      delete item;
//      return nullptr;
//    }
//    _itemsMap.insert(timeBlockName, list);

//    if (!isEventBlock) {
//      item->setColor(getNewColor(_colorIndex));
//      _colorIndex = (_colorIndex + 1) % _colorsList.size();
//    }
//  } else {
//    list = iter.value();
//    if (!isEventBlock) {
//      item->setColor(list->at(0)->itemColor());
//    }
//  }

//  list->append(item);

//  return item;
//}

TimeBlockItem *TimeLineRow::addTimeBlock(const TimeBlockData *blockData) {
  TimeBlockItem *item = TimeBlockItemPool::instance()->allocate();
  if (item == nullptr) {
    return nullptr;
  }

  item->setVisible(true);
  item->setTimeBlockData(blockData);

  addToGroup(item);
  item->setPos(0, 0);
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

  //  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
  //    QList<TimeBlockItem *> *list = iter.value();
  //    for (TimeBlockItem *item : *list) {
  //      r = item->rect();
  //      r.setHeight(h);
  //      item->setRect(r);
  //    }
  //  }
}

void TimeLineRow::moveTimeBlockLr(qreal offsetX) {
  //  for (auto iter = _itemsMap.begin(); iter != _itemsMap.end(); ++iter) {
  //    QList<TimeBlockItem *> *list = iter.value();
  //    for (TimeBlockItem *item : *list) {
  //      item->setX(item->x() + offsetX);
  //    }
  //  }
}

void TimeLineRow::recalculateTimeBlock(int64_t &base, int timeRatio, int scale,
                                       qreal rowHeaderWidth) {
  qreal h = _backgroud.rect().height();

  removeFromGroup(&_headerText);
  removeFromGroup(&_backgroud);

  QList<QGraphicsItem *> children = childItems();
  addToGroup(&_headerText);
  addToGroup(&_backgroud);
  for (QGraphicsItem *iter : children) {
    bool isSpecialItem;
    int64_t start, end;

    TimeBlockItem *item = static_cast<TimeBlockItem *>(iter);

    isSpecialItem = false;

    start = item->startTimeStamp();
    end = item->endTimeStamp();

    if (start < 1LL) {
      start = end;
      isSpecialItem = true;
    }

    //      if(start < _renderBegTime) continue;

    if (item->format() == TimeBlockData::Format::Format8) {
      end -= start;  // to length
    } else {
      start = end - item->latency();
      end = item->latency();
    }
    end /= timeRatio;

    //      if(end > _timeLineLength) continue;

    start -= base;  // to zero coordinate

    end *= scale;
    start *= scale;

    start /= timeRatio;

    if (isSpecialItem) {
      end = 1;
    }

    start += rowHeaderWidth;

    int offsetH = item->eventId();
    offsetH <<= 3;

    item->setRect(0, 0, end, h - offsetH);
    item->setPos(start, offsetH);
  }
}

void TimeLineRow::clear(void) {
  _headerText.setToolTip(QString(""));
  _headerText.setPlainText(QString(""));

  removeFromGroup(&_headerText);
  removeFromGroup(&_backgroud);

  QList<QGraphicsItem *> children = childItems();
  addToGroup(&_headerText);
  addToGroup(&_backgroud);

  QGraphicsScene *s = scene();

  TimeBlockItemPool *p = TimeBlockItemPool::instance();
  for (auto item : children) {
    removeFromGroup(item);
    s->removeItem(item);

    p->deallocate(static_cast<TimeBlockItem *>(item));
  }
}

void TimeLineRow::filterEventId(int eventId) {
  removeFromGroup(&_headerText);
  removeFromGroup(&_backgroud);

  QList<QGraphicsItem *> children = childItems();
  addToGroup(&_headerText);
  addToGroup(&_backgroud);
  for (QGraphicsItem *iter : children) {
    TimeBlockItem *item = static_cast<TimeBlockItem *>(iter);

    if (eventId == -1) {
      item->setVisible(true);
    } else if (item->eventId() == eventId) {
      item->setVisible(true);
    } else {
      item->setVisible(false);
    }
  }
}

#undef getNewColor
