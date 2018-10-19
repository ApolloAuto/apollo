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

#include "./arrow.h"
#include "./composite_item.h"

#include <cmath>
#include <iostream>

#include <QPainter>
#include <QPen>

constexpr double PI = 3.14159265358979323846264338327950288419716939937510;
constexpr double ArrowSize = 10;

Arrow::Arrow(CompositeItem *startItem, CompositeItem *endItem,
             QGraphicsItem *parent)
    : QGraphicsLineItem(parent),
      start_item_(startItem),
      end_item_(endItem),
      polygon_() {
  setPen(QPen(Qt::black, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
}

QRectF Arrow::boundingRect() const {
  qreal extra = (pen().width() + 20) / 2.0;

  return QRectF(line().p1(), QSizeF(line().p2().x() - line().p1().x(),
                                    line().p2().y() - line().p1().y()))
      .normalized()
      .adjusted(-extra, -extra, extra, extra);
}

QPainterPath Arrow::shape() const {
  QPainterPath path = QGraphicsLineItem::shape();
  path.addPolygon(polygon_);
  return path;
}

void Arrow::updatePosition() {
  QLineF line(mapFromItem(start_item_, 0, 0), mapFromItem(end_item_, 0, 0));
  setLine(line);
}

QPointF Arrow::Intersect(const QPolygonF &polygon, const QPointF &point,
                         const QLineF &line) {
  QPointF p1 = polygon.first() + point;
  QPointF p2;
  QPointF intersectPoint(0, 0);
  QLineF polyLine;
  for (int i = 1; i < polygon.count(); ++i) {
    p2 = polygon.at(i) + point;
    polyLine = QLineF(p1, p2);
    QLineF::IntersectType intersectType =
        polyLine.intersect(line, &intersectPoint);
    if (intersectType == QLineF::BoundedIntersection) break;
    p1 = p2;
  }
  return intersectPoint;
}

void Arrow::paint(QPainter *painter, const QStyleOptionGraphicsItem *,
                  QWidget *) {
  if (start_item_->collidesWithItem(end_item_)) {
    return;
  }

  painter->setPen(pen());
  painter->setBrush(pen().color());

  QPointF startPoint =
      start_item_->pos() + QPointF(start_item_->boundingRect().width() / 2,
                                   start_item_->boundingRect().height() / 2);
  QPointF endPoint =
      end_item_->pos() + QPointF(end_item_->boundingRect().width() / 2,
                                 end_item_->boundingRect().height() / 2);

  QLineF centerLine(startPoint, endPoint);

  endPoint = Intersect(end_item_->polygon(), end_item_->pos(), centerLine);
  startPoint =
      Intersect(start_item_->polygon(), start_item_->pos(), centerLine);

  setLine(QLineF(endPoint, startPoint));

  double angle = std::acos(line().dx() / line().length());
  if (line().dy() >= 0) angle = (PI * 2) - angle;

  QPointF arrowP1 = line().p1() + QPointF(std::sin(angle + PI / 3) * ArrowSize,
                                          std::cos(angle + PI / 3) * ArrowSize);
  QPointF arrowP2 =
      line().p1() + QPointF(std::sin(angle + PI - PI / 3) * ArrowSize,
                            std::cos(angle + PI - PI / 3) * ArrowSize);

  polygon_.clear();
  polygon_ << line().p1() << arrowP1 << arrowP2;

  painter->drawLine(line());
  painter->drawPolygon(polygon_);
}
