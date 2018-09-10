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

#include "time_line_scene.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsView>
#include <QMessageBox>
#include <iostream>
#include "time_block_item.h"
#include "time_line_axis.h"
#include "time_line_legend.h"
#include "time_line_legend_item.h"
#include "time_line_table.h"

namespace {
QList<QColor> _eventColorsList{QRgb(0x89ff5f), QRgb(0x5f89ff), QRgb(0xfeef59),
                               QRgb(0xffa55f), QRgb(0xef59fe)};

#define getEventColor(i) _eventColorsList.at(i)
}

TimeLineScene::TimeLineScene(QObject* parent)
    : QGraphicsScene(parent),

      _axisScale(5),
      _axisSparsity(10),
      _headerWidth(150),
      _rowHeight(30),
      _rowOffset(10),

      _currentProcessorIndex(-1),
      _timeRatio(1000000),
      _timeBegin(0),
      _timeEnd(20000000000LL),
      _newAxis(new TimeLineAxis()),
      _processorTable(new TimeLineTable),
      _taskTable(new TimeLineTable),
      _taskLegend(new TimeLineLegend),
      _taskProcessorName(new QGraphicsTextItem),
      _timeGrid(new QGraphicsItemGroup) {
  addItem(_newAxis);
  addItem(_processorTable);
  addItem(_taskTable);
  addItem(_taskLegend);
  addItem(_taskProcessorName);
  addItem(_timeGrid);

  _timeGrid->setPos(_headerWidth, 0);
  _newAxis->setPos(0, 0.0f);
  _processorTable->setPos(0, _rowHeight);
  _taskTable->setPos(0, _rowHeight * 34);
  _taskProcessorName->setPos(0, _taskTable->y() + _rowHeight * 10);
  _taskLegend->setPos(50, _taskProcessorName->y());

  _newAxis->setHeaderText(tr("Processor"));
  _newAxis->adjustHeaderTextPos(_headerWidth, _rowHeight);

  _processorTable->setRowHeaderWidth(_headerWidth);
  _taskTable->setRowHeaderWidth(_headerWidth);
  _processorTable->setRowHeight(_rowHeight);
  _taskTable->setRowHeight(_rowHeight);
  _processorTable->setRowOffset(_rowOffset);
  _taskTable->setRowOffset(_rowOffset);

  _processorTable->setWidth(timeLength2Width(20000));
  _taskTable->setWidth(timeLength2Width(20000));

  _taskLegend->setLegendItemsCount(4);
  for (int i = 0; i < 4; ++i) {
    TimeLineLegendItem* item = _taskLegend->itemByIndex(i);
    item->setLegendColorText(getEventColor(i), tr("EventId:%1").arg(i));
  }

  _taskLegend->adjustItemsPos();

  _processorTable->setVisible(false);
  _taskTable->setVisible(false);
  _taskProcessorName->setVisible(false);
  _taskLegend->setVisible(false);
  _timeGrid->setVisible(false);

  _newAxis->draw(_headerWidth, 20000, _rowHeight, _axisScale, _axisSparsity);
}

void TimeLineScene::setRowHeight(int h) {
  qreal y = _taskTable->y();
  y -= _processorTable->y();
  y -= _processorTable->height();

  _taskTable->setY(y + _processorTable->y() +
                   _processorTable->rowCount() * (h + _rowOffset));
  _processorTable->setY(h);

  _processorTable->setRowHeight(h);
  _taskTable->setRowHeight(h);

  _rowHeight = h;

  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);
}

void TimeLineScene::setTimeEnd(int64_t l) {
  _timeEnd = l;

  l -= _timeBegin;
  l += _timeRatio - 1;
  l /= _timeRatio;

  if (l < 1) return;

  l *= axisScale();
  l += rowHeaderWidth();

  QRectF rect = sceneRect();
  if (l == rect.width()) return;

  rect.setX(0);
  rect.setY(0);
  rect.setWidth(l);
  rect.setHeight(l);

  setSceneRect(rect);

  _processorTable->setWidth(l);
  _taskTable->setWidth(l);
  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);
}

void TimeLineScene::setSceneScale(int s) {
  _axisScale = s;

  int w = timeLength2Width(timeLineLength());

  setSceneRect(0, 0, w, w);

  _processorTable->setWidth(w);
  _taskTable->setWidth(w);

  _processorTable->recalculateTimeBlock(_timeBegin, _timeRatio, _axisScale);
  _taskTable->recalculateTimeBlock(_timeBegin, _timeRatio, _axisScale);


  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);
  destroyTimeGrid();
  createTimeGrid();
}

void TimeLineScene::setSceneSparsity(int s) {
  _axisSparsity = s;
  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);
  destroyTimeGrid();
  createTimeGrid();
}

void TimeLineScene::setRowHeaderWidth(int w) {
  qreal offsetX = w - rowHeaderWidth();
  if (offsetX == 0.0f) return;

  QRectF rect = sceneRect();
  offsetX += rect.width();

  rect.setWidth(offsetX);
  setSceneRect(rect);

  _processorTable->setWidth(offsetX);
  _taskTable->setWidth(offsetX);
  _processorTable->setRowHeaderWidth(w);
  _taskTable->setRowHeaderWidth(w);

  _headerWidth = w;
  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);
  _newAxis->adjustHeaderTextPos(_headerWidth, _rowHeight);
}

TimeLineRow* TimeLineScene::addOneTimeLineByRowIndex(int rowIndex,
                                                     TimeLineTable* table) {
  if (!table->isVisible()) {
    table->setVisible(true);
  }
  if (rowIndex + 1 > table->rowCount()) {
    if (!table->setRowCount(rowIndex + 1)) return nullptr;
  }

  return table->rowAt(rowIndex);
}

TimeBlockItem* TimeLineScene::addTimeBlock(TimeLineRow* row,
                                           const QString& blockName,
                                           int64_t& start, int64_t& end,
                                           bool isEventBlock) {
  if (row == nullptr) return nullptr;
  TimeBlockItem* item = row->addTimeBlock(blockName, start, end, isEventBlock);
  if (item == nullptr) return nullptr;

  bool isSpecialItem = false;
  if (start < 1LL) {
    start = end;
    isSpecialItem = true;
  }

  end -= start;         // to width
  start -= _timeBegin;  // to zero coordinate

  end *= _axisScale;
  start *= _axisScale;

  end /= _timeRatio;
  start /= _timeRatio;

  if(isSpecialItem){
      end = 1;
      item->setZValue(1);
  }

  start += rowHeaderWidth();

  item->setRect(0, 0, end, _rowHeight);
  item->setPos(start, 0);
  return item;
}

void TimeLineScene::setTimeRatio(int tr) {
  int64_t l = _timeEnd - _timeBegin;
  l /= tr;

  l += _axisSparsity - 1;
  l /= _axisSparsity;
  l *= _axisSparsity;

  l += _headerWidth;

  _timeRatio = tr;

  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);

  _processorTable->setWidth(l);
  _taskTable->setWidth(l);

  _processorTable->recalculateTimeBlock(_timeBegin, _timeRatio, _axisScale);
  _taskTable->recalculateTimeBlock(_timeBegin, _timeRatio, _axisScale);
}

void TimeLineScene::sceneDoubleClicked(QGraphicsItem* e, QPointF& scenePos) {
  if (e == nullptr) return;

  switch (e->type()) {
    case TimeBlockItem::Type: {
      return showTimeBlockInfo(static_cast<TimeBlockItem*>(e));
    }
    default:
      showProcessorTasks(scenePos);
  }
}

void TimeLineScene::showProcessorTasks(QPointF& p) {
  if (p.x() > rowHeaderWidth()) return;

  int rH = p.y();
  rH += _rowOffset;

  rH /= (_processorTable->rowHeight() + _rowOffset);

  if (rH > _processorTable->rowCount()) return;
  --rH;  // to index
  if (rH < 0) {
    _taskTable->setVisible(false);
    _taskProcessorName->setVisible(false);
    _taskLegend->setVisible(false);
  } else {
    if (rH == _currentProcessorIndex) return;
    _currentProcessorIndex = rH;

    _taskTable->setPos(0, (_rowHeight << 1) + _processorTable->height());

    TimeLineRow* row = _processorTable->rowAt(rH);
    int c = row->taskCount();

    _taskTable->setVisible(true);
    _taskProcessorName->setVisible(true);
    _taskLegend->setVisible(true);

    if (_taskTable->rowCount() != c) {
      _taskTable->setRowCount(c);

      _taskProcessorName->setPos(
          0, _taskTable->y() + _taskTable->height() + _rowHeight);
      _taskProcessorName->setPlainText(
          tr("Processor: %1").arg(row->rowHeaderText()));
      _taskLegend->setPos(_taskProcessorName->boundingRect().width() + 15,
                          _taskProcessorName->y());

      adjustTimeGrid();
    }

    int index = 0;
    for (auto iter = row->_itemsMap.begin(); iter != row->_itemsMap.end();
         ++iter, ++index) {
      TimeLineRow* taskRow = _taskTable->rowAt(index);
      taskRow->clear();
      taskRow->setRowHeaderText(iter.key());
      taskRow->adjustHeaderTextPos(_headerWidth, _rowHeight);

      QList<TimeBlockItem*>* list = iter.value();
      for (TimeBlockItem* item : *list) {
        int64_t s;
        int64_t e;

        s = item->startTimeStamp();
        e = item->endTimeStamp();

        TimeBlockItem* taskBlockItem =
            addTimeBlock(taskRow, tr("%1").arg(item->eventId()), s, e, true);

        if (taskBlockItem) {
          taskBlockItem->setEventId(item->eventId());
          taskBlockItem->setColor(getEventColor(item->eventId()));
        }
      }
    }
  }
}

void TimeLineScene::showTimeBlockInfo(TimeBlockItem* item) {
  int64_t s = item->startTimeStamp();
  int64_t e = item->endTimeStamp();
  int64_t b = timeBegin();

  QMessageBox::information(nullptr, QObject::tr("TimeStamp Information"),
                           QObject::tr("Name: (%1)\n"
                                       "TimeBase(b): %2 NS\n"
                                       "StartTime(s): %3 NS\n"
                                       "EndTime(e): %4 NS\n"
                                       "(s - b): %5 NS\n"
                                       "(e - s): %6 NS\n")
                               .arg(item->toolTip())
                               .arg(b)
                               .arg(s)
                               .arg(e)
                               .arg(s - b)
                               .arg(e - s));
}

int TimeLineScene::processorCount(void) const {
  return _processorTable->rowCount();
}

void TimeLineScene::keepStill(qreal v) {
  _timeGrid->setX(_headerWidth + v);
  _taskProcessorName->setX(v);
  _taskLegend->setX(v + _taskProcessorName->boundingRect().width() + 15);
}

void TimeLineScene::createTimeGrid() {
  QGraphicsView* view = views().at(0);

  int s = _axisSparsity;
  int length = view->width();
  qreal h;
  if (_taskTable->isVisible()) {
    h = _taskTable->y() + _taskTable->height();
  } else {
    h = _processorTable->y() + _processorTable->height();
  }

  QPen p(Qt::black);

  int i = 0;
  for (; i < length; i += s) {
    QGraphicsLineItem* l = new QGraphicsLineItem();
    if (l) {
      _timeGrid->addToGroup(l);
      l->setPen(p);
      l->setZValue(10);
      l->setLine(0, 0, 0, h);
      l->setPos(i * _axisScale, 0);
    } else {
      break;
    }
  }
  if (i < length) {
    destroyTimeGrid();
  }
}

void TimeLineScene::destroyTimeGrid(void) {
  QList<QGraphicsItem*> list = _timeGrid->childItems();
  for (auto item : list) {
    delete item;
  }
}

void TimeLineScene::adjustTimeGrid(void) {
  qreal h;
  if (_taskTable->isVisible()) {
    h = _taskTable->y() + _taskTable->height();
  } else {
    h = _processorTable->y() + _processorTable->height();
  }

  QPointF p;
  QList<QGraphicsItem*> list = _timeGrid->childItems();
  for (auto item : list) {
    p = item->pos();
    static_cast<QGraphicsLineItem*>(item)->setLine(0, 0, 0, h);
    item->setPos(p);
  }
}
