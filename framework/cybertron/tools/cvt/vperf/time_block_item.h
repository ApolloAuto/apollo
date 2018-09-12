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

#ifndef TIMEBLOCKITEM_H
#define TIMEBLOCKITEM_H

#include <QBrush>
#include <QGraphicsRectItem>
#include <QPen>
#include "perf_data_base.h"

class TimeBlockItem : public QGraphicsRectItem {
 public:
  explicit TimeBlockItem(QGraphicsItem *parent = nullptr);
  explicit TimeBlockItem(const TimeBlockData *data,
                         QGraphicsItem *parent = nullptr);

  enum { Type = QGraphicsRectItem::UserType + QGraphicsRectItem::Type };
  virtual int type() const override { return Type; }

  void setColor(const QColor &color) {
    setPen(QPen(color));
    setBrush(QBrush(color));
  }

  void setColor(Qt::PenStyle style) {
    setPen(QPen(style));
    setBrush(QBrush(style));
  }

  QColor itemColor(void) const { return brush().color(); }

  void highlight(void) {
    QBrush b = brush();
    b.setStyle(Qt::DiagCrossPattern);
    setBrush(b);
  }

  void clearHighlight(void) {
    QBrush b = brush();
    b.setStyle(Qt::SolidPattern);
    setBrush(b);
  }

  int64_t startTimeStamp(void) const { return dataPtr->startTimeStamp(); }
  int64_t endTimeStamp(void) const { return dataPtr->endTimeStamp(); }
  int eventId(void) const { return dataPtr->eventId(); }

  int preEventId(void) const { return dataPtr->preEventId(); }
  int tryFetchHolderValue(void) const { return dataPtr->tryFetchHolderValue(); }
  std::int64_t preEndTime(void) const { return dataPtr->preEndTime(); }
  TimeBlockData::Format format(void) const { return dataPtr->format(); }

  int latency(void) const { return dataPtr->latency(); }

  void setTimeBlockData(const TimeBlockData *ptr) { dataPtr = ptr; }

 private:
  const TimeBlockData *dataPtr;
};

#endif  // TIMEBLOCKITEM_H
