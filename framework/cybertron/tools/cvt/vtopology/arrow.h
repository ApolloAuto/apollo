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

#ifndef TOOLS_CVT_VTOPOLOGY_ARROW_H_
#define TOOLS_CVT_VTOPOLOGY_ARROW_H_

#include <QColor>
#include <QGraphicsLineItem>
#include <QPainterPath>
#include <QPen>
#include <QRectF>
#include <QString>

class CompositeItem;

class Arrow : public QGraphicsLineItem {
 public:
  enum { Type = UserType + 9 };

  Arrow(CompositeItem *startItem, CompositeItem *endItem,
        QGraphicsItem *parent = nullptr);
  ~Arrow() {
    start_item_ = nullptr;
    end_item_ = nullptr;
  }

  int type() const override { return Type; }
  QRectF boundingRect() const override;
  QPainterPath shape() const override;

  CompositeItem *start_item() const { return start_item_; }
  CompositeItem *end_item() const { return end_item_; }

  void updatePosition();

 protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = 0) override;

 private:
  static QPointF Intersect(const QPolygonF &polygon, const QPointF &point,
                           const QLineF &line);

  CompositeItem *start_item_;
  CompositeItem *end_item_;
  QPolygonF polygon_;
};

#endif  // TOOLS_CVT_VTOPOLOGY_ARROW_H_
