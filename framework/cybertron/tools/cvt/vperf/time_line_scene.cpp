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

#include "time_line_scene.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsView>
#include <QHash>
#include <QMessageBox>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "time_block_item.h"
#include "time_line_axis.h"
#include "time_line_legend.h"
#include "time_line_legend_item.h"
#include "time_line_table.h"
#include "time_line_view.h"

namespace {

constexpr int EventIdCount = 5;

QList<QRgb> _eventColorsList{QRgb(0x89ff5f), QRgb(0x5f89ff), QRgb(0xfeef59),
                             QRgb(0xffa55f), QRgb(0xef59fe), QRgb(0xef9922)};

#define getEventColor(i) _eventColorsList.at(i)

// QList<QRgb> _taskColorsList{

//    QRgb(0x209fdf), QRgb(0x99ef53), QRgb(0xf6a625),
//    QRgb(0x6defd5), QRgb(0xbf593e),

//    QRgb(0xdf209f), QRgb(0x5399ef), QRgb(0x25f6a6),
//    QRgb(0xd56def), QRgb(0x3ebf59),

//    QRgb(0x9fdf20), QRgb(0xef5399), QRgb(0xa625f6),
//    QRgb(0xefd56d), QRgb(0x593ebf),
//};

constexpr float constr = std::atan(0.707106781) * std::cos(45.0);
constexpr float constg = std::atan(0.707106781) * std::sin(45.0);
constexpr float radius = 12436031.25f;

class ColorSys {
  struct InnerColor {
    InnerColor* _next;
    std::string _name;
    QRgb _color;

    explicit InnerColor(const std::string& name, QRgb c)
        : _next(nullptr), _name(name), _color(c) {}
    ~InnerColor() { _next = nullptr; }
  };

  int _colorIndex;
  std::vector<InnerColor*> _colorList;

  QRgb getNewTaskColor() {
    static float r = radius;

    int rv = rand();
    std::srand(std::time(nullptr) + static_cast<int>(r));

    float verticalR = rv / RAND_MAX;
    verticalR -= 0.5f;
    verticalR *= 2.0f;

    verticalR *= rv / 53 + rv % 191;
    r = verticalR + radius;

    rv %= 360;

    float cr = constr * r;
    float cg = constg * r;
    float cb = r / 1.732050808f;

    cr += verticalR * std::cos(rv);
    cg += verticalR * std::sin(rv);

    cr *= 1.3f;
    cg *= 1.3f;

    cr /= r;
    cg /= r;
    cr *= 255;
    cg *= 255;

    return qRgb(static_cast<int>(cr), static_cast<int>(cg),
                static_cast<int>(cb));
  }

 public:
  static constexpr uint ColorSysSize = 13U;

  explicit ColorSys(void) : _colorIndex(0), _colorList() {
    _colorList.resize(ColorSysSize, nullptr);
  }
  ~ColorSys(void) { clear(); }

  const QRgb* getColorByName(const std::string& name) {
    uint v = qHash(QString(name.c_str()));
    v %= ColorSysSize;

    const QRgb* ret = nullptr;
    InnerColor* header = _colorList.at(v);
    if (header == nullptr) {
      InnerColor* c = new InnerColor(name, getNewTaskColor());
      if (c) {
        _colorList[v] = c;
        ret = &c->_color;
      }
    } else {
      InnerColor* c = header;
      for (; c != nullptr; c = c->_next) {
        if (c->_name == name) {
          ret = &c->_color;
          break;
        }
      }

      if (!c) {
        c = new InnerColor(name, getNewTaskColor());
        if (c) {
          c->_next = header;

          _colorList[v] = c;

          ret = &c->_color;
        }
      }
    }

    return ret;
  }

  void clear(void) {
    for (uint i = 0; i < ColorSysSize; ++i) {
      InnerColor* header = _colorList.at(i);
      while (header) {
        InnerColor* p = header;
        header = header->_next;

        delete p;
      }
      _colorList[i] = nullptr;
    }
  }
};

ColorSys _colorSys;
}

TimeLineScene::TimeLineScene(QObject* parent)
    : QGraphicsScene(parent),

      _axisScale(5),
      _axisSparsity(10),
      _headerWidth(150),
      _rowHeight(50),
      _rowOffset(10),

      _currentProcessorIndex(-1),
      _timeRatio(1000000),
      _timeLineLength(20000),
      _viewXOffset(0), /*_timeBegin(0),*/
      /*_timeEnd(20000000000LL),*/ _renderBegTime(0),
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

  _taskLegend->setLegendItemsCount(EventIdCount);
  for (int i = 0; i < EventIdCount; ++i) {
    TimeLineLegendItem* item = _taskLegend->itemByIndex(i);
    item->setLegendColorText(getEventColor(i), tr("EventId:%1").arg(i + 1));
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

// void TimeLineScene::setTimeEnd(int64_t l) {
//  _timeEnd = l;

//  l -= _timeBegin;
//  l += _timeRatio - 1;
//  l /= _timeRatio;

//  if (l < 1)
//    return;

//  l *= axisScale();
//  l += rowHeaderWidth();

//  QRectF rect = sceneRect();
//  if (l == rect.width())
//    return;

//  rect.setX(0);
//  rect.setY(0);
//  rect.setWidth(l);
//  rect.setHeight(l);

//  setSceneRect(rect);

//  _processorTable->setWidth(l);
//  _taskTable->setWidth(l);
//  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
//                   _axisSparsity);
//}

void TimeLineScene::setSceneScale(int s) {
  _axisScale = s;

  int w = timeLength2Width(timeLineLength());

  setSceneRect(0, 0, w, w);

  _processorTable->setWidth(w);
  _taskTable->setWidth(w);

  if (_processorTable->isVisible()) {
    _processorTable->recalculateTimeBlock(_renderBegTime, _timeRatio,
                                          _axisScale);
  }

  if (_taskTable->isVisible()) {
    _taskTable->recalculateTimeBlock(_renderBegTime, _timeRatio, _axisScale);
  }

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

void TimeLineScene::drawProcessorData(const PerfBlockDatabase* blockData) {
  for (int i = 0; i < _processorTable->rowCount(); ++i) {
    const ProcessorData* procData = blockData->processorData(i);
    TimeLineRow* row = _processorTable->rowAt(i);
    row->clear();
    row->setRowHeaderText(QObject::tr("%1").arg(i));
    row->setProcessorData(procData);
    row->adjustHeaderTextPos(_headerWidth, _rowHeight);

    const std::map<const std::string, TaskData*>& rowData = procData->data();
    auto iter = rowData.cbegin();
    for (; iter != rowData.cend(); ++iter) {
      const TaskData* taskData = iter->second;
      drawTaskData(taskData, row, QString(iter->first.c_str()),
                   _colorSys.getColorByName(iter->first));
    }
  }
}

void TimeLineScene::drawTaskData(const TaskData* taskData, TimeLineRow* row,
                                 const QString& name, const QRgb* c) {
  const std::list<TimeBlockData*>& blockData = taskData->data();

  for (auto blockIter = blockData.cbegin(); blockIter != blockData.cend();
       ++blockIter) {
    bool isSpecialItem;
    int64_t start, end;
    const TimeBlockData* block = *blockIter;

    isSpecialItem = false;

    start = block->startTimeStamp();
    end = block->endTimeStamp();

    if (start < 1LL) {
      start = end;
      isSpecialItem = true;
    }

    if (start < _renderBegTime) continue;

    if (block->format() == TimeBlockData::Format::Format8) {
      end -= start;  // to length
    } else {
      start = end - block->latency();
      end = block->latency();
    }
    end /= _timeRatio;

    if (end > _timeLineLength) continue;

    TimeBlockItem* item = row->addTimeBlock(block);

    if (item == nullptr) continue;

    start -= _renderBegTime;  // to zero coordinate

    end *= _axisScale;
    start *= _axisScale;

    start /= _timeRatio;

    if (isSpecialItem) {
      end = 1;
    }

    start += rowHeaderWidth();
    int h = block->eventId();
    h <<= 3;

    item->setRect(0, 0, end, _rowHeight - h);
    item->setPos(start, h);
    item->setZValue(block->eventId());
    item->setToolTip(name);

    if (c) {
      item->setColor(*c);
    } else {
      item->setColor(getEventColor(item->eventId()));
    }
  }
}

void TimeLineScene::setTimeRatio(int tr) {
  int64_t l = _timeLineLength;
  l *= tr;
  l /= _timeRatio;

  l += _headerWidth;
  _timeRatio = tr;
  _timeLineLength = l;

  setSceneRect(0, 0, l, l);

  _newAxis->redraw(_headerWidth, timeLineLength(), _rowHeight, _axisScale,
                   _axisSparsity);

  _processorTable->setWidth(l);
  _taskTable->setWidth(l);

  _processorTable->recalculateTimeBlock(_renderBegTime, _timeRatio, _axisScale);
  _taskTable->recalculateTimeBlock(_renderBegTime, _timeRatio, _axisScale);
}

void TimeLineScene::sceneDoubleClicked(QGraphicsItem* e, QPointF& scenePos) {
  if (e == nullptr) return;

  switch (e->type()) {
    case TimeBlockItem::Type: {
      return showTimeBlockInfo(static_cast<TimeBlockItem*>(e));
    }
    default:
      showProcessorTasks(e, scenePos);
  }
}

void TimeLineScene::showProcessorTasks(QGraphicsItem* e, QPointF& p) {
  //  if (p.x() > rowHeaderWidth()) return;

  int rH = p.y();
  rH += _rowOffset;

  rH /= (_processorTable->rowHeight() + _rowOffset);

  if (rH > _processorTable->rowCount()) {
    return;
  }
  --rH;

  if (rH < 0) {
    _taskTable->setVisible(false);
    _taskProcessorName->setVisible(false);
    _taskLegend->setVisible(false);
    _currentProcessorIndex = -1;
  } else {
    if (rH == _currentProcessorIndex) {
      return;
    }

    TimeLineRow* row = _processorTable->rowAt(rH);
    if (&row->_backgroud != e && &row->_headerText != e) {
      return;
    }

    _currentProcessorIndex = rH;
    _taskTable->setVisible(true);
    _taskProcessorName->setVisible(true);
    _taskLegend->setVisible(true);

    return drawTaskTable();
  }
}

void TimeLineScene::drawTaskTable(void) {
  _taskTable->setPos(0, (_rowHeight << 1) + _processorTable->height());

  TimeLineRow* row = _processorTable->rowAt(_currentProcessorIndex);
  int c = row->taskCount();

  _taskTable->setRowCount(c);
  adjustTimeGrid();

  _taskProcessorName->setPlainText(
      tr("Processor: %1").arg(row->rowHeaderText()));
  _taskProcessorName->setPos(
      _viewXOffset, _taskTable->y() + _taskTable->height() + _rowHeight);
  _taskLegend->setPos(
      _viewXOffset + _taskProcessorName->boundingRect().width() + 15,
      _taskProcessorName->y());

  int index = 0;

  const std::map<const std::string, TaskData*>& rowData =
      row->_procDataPtr->data();
  for (auto iter = rowData.cbegin(); iter != rowData.cend(); ++iter, ++index) {
    TimeLineRow* taskRow = _taskTable->rowAt(index);
    taskRow->clear();
    taskRow->setRowHeaderText(iter->first);
    taskRow->adjustHeaderTextPos(_headerWidth, _rowHeight);

    drawTaskData(iter->second, taskRow, QString(iter->first.c_str()));
  }

  qreal h = _taskProcessorName->y() + 20;

  for (QGraphicsView* v : views()) {
    TimeLineView* tv = static_cast<TimeLineView*>(v);
    tv->setTargetHeight(h);
  }
}

void TimeLineScene::showTimeBlockInfo(TimeBlockItem* item) {
  int64_t s = item->startTimeStamp();
  int64_t e = item->endTimeStamp();
  int64_t b = PerfDatabase::instance()->startTimeStamp();

  int latency = -1;
  if (item->format() == TimeBlockData::Format::Format11) {
    latency = item->latency();
  }

  QMessageBox::information(nullptr, QObject::tr("TimeStamp Information"),
                           QObject::tr("Name: (%1)\n"
                                       "TimeBase(b): %2 NS\n"
                                       "StartTime(s): %3 NS\n"
                                       "EndTime(e): %4 NS\n"
                                       "(s - b): %5 NS\n"
                                       "(e - s): %6 NS\n"
                                       "latency: %7 NS\n"
                                       "EventId: %8")
                               .arg(item->toolTip())
                               .arg(b)
                               .arg(s)
                               .arg(e)
                               .arg(s - b)
                               .arg(e - s)
                               .arg(latency)
                               .arg(item->eventId()));
}

int TimeLineScene::processorCount(void) const {
  return _processorTable->rowCount();
}

void TimeLineScene::keepStill(qreal v) {
  _viewXOffset = v;
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

void TimeLineScene::resetAll(void) {
  _processorTable->setVisible(false);
  _taskTable->setVisible(false);
  _taskLegend->setVisible(false);
  _taskProcessorName->setVisible(false);
  _timeGrid->setVisible(false);
  _currentProcessorIndex = -1;
  _colorSys.clear();
}

void TimeLineScene::drawBlockDatabase(const PerfBlockDatabase* blockDB) {
  _blockDatabasePtr = blockDB;

  _renderBegTime = blockDB->startTimeStamp() - _timeRatio;

  int64_t tmp = _renderBegTime - PerfDatabase::instance()->startTimeStamp();
  tmp /= _timeRatio;

  // rounding down
  tmp /= _axisSparsity;
  tmp *= _axisSparsity;

  _newAxis->setBaseTime(tmp, _axisSparsity << 1);

  _processorTable->setRowCount(blockDB->processorCount());
  drawProcessorData(blockDB);
  _processorTable->setVisible(true);

  qreal h = _processorTable->y() + _processorTable->height();

  if (_taskTable->isVisible()) {
    _taskTable->setRowCount(blockDB->processorCount());
    drawTaskTable();

    h = _taskProcessorName->y();
  }

  h += 20;

  for (QGraphicsView* v : views()) {
    TimeLineView* tv = static_cast<TimeLineView*>(v);
    tv->setTargetHeight(h);
  }
}

void TimeLineScene::filterProcessorByEventId(int processorIndex, int eventId) {
  TimeLineRow* row = _processorTable->rowAt(processorIndex);
  if (eventId >= EventIdCount) eventId = -1;
  row->filterEventId(eventId);
}
