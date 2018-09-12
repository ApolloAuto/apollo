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

#ifndef TIMELINELEGENDITEM_H
#define TIMELINELEGENDITEM_H

#include <QBrush>
#include <QGraphicsItemGroup>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QPen>

class TimeLineLegend;

class TimeLineLegendItem : public QGraphicsItemGroup {
 public:
  enum {
    Type = QGraphicsItemGroup::UserType + QGraphicsItemGroup::Type +
           QGraphicsRectItem::Type + QGraphicsTextItem::Type
  };

  explicit TimeLineLegendItem(QGraphicsItem *parent = nullptr);
  ~TimeLineLegendItem();

  int type() const { return Type; }
  QRectF boundingRect() const override;

  void setLegendColor(const QColor &c) {
    _legendColor.setBrush(QBrush(c));
    _legendColor.setPen(QPen(c));
  }

  void setLegendText(const QString &text) { _legendText.setPlainText(text); }

  void setLegendColorText(const QColor &c, const QString &text) {
    setLegendColor(c);
    setLegendText(text);
  }

  qreal itemWidth(void) const { return boundingRect().width(); }

 private:
  QGraphicsRectItem _legendColor;
  QGraphicsTextItem _legendText;

  friend class TimeLineLegend;
};

#endif  // TIMELINELEGENDITEM_H
