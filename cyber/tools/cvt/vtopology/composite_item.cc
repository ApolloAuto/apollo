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

#include "./composite_item.h"
#include "./arrow.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QPainterPath>
#include <iostream>

CompositeItem::CompositeItem(ItemType itemType, const QString &text,
                             QGraphicsItem *parent)
    : QGraphicsItemGroup(parent),
      item_type_(itemType),
      old_pen_(Qt::black),
      current_pen_(Qt::black),
      text_(text),
      shape_() {
  QRectF rect = text_.boundingRect();
  QPainterPath path;

  switch (itemType) {
    case ItemType::Node:
      rect.adjust(-8, -8, 8, 8);
      path.addEllipse(rect);
      break;
    case ItemType::Channel:
    default:
      path.addRect(rect);
      break;
  }

  shape_.setPolygon(path.toFillPolygon());

  addToGroup(&text_);
  addToGroup(&shape_);
  setFocusProxy(&shape_);

  setFlag(QGraphicsItem::ItemIsMovable);
  setFlag(QGraphicsItem::ItemIsSelectable);
}

QPolygonF CompositeItem::polygon() const { return shape_.polygon(); }

void CompositeItem::paint(QPainter *painter,
                          const QStyleOptionGraphicsItem *option,
                          QWidget *widget) {
  text_.setDefaultTextColor(current_pen_.color());
  text_.paint(painter, option, widget);

  shape_.setPen(current_pen_);
  shape_.paint(painter, option, widget);
}

void CompositeItem::hoverEnterEvent(QGraphicsSceneHoverEvent * /*event*/) {
  old_pen_ = current_pen_;
  current_pen_.setColor(Qt::red);
}

void CompositeItem::hoverLeaveEvent(QGraphicsSceneHoverEvent * /*event*/) {
  current_pen_ = old_pen_;
}

void CompositeItem::removeArrow(Arrow *arrow) {
  int index = arrows_.indexOf(arrow);
  if (index != -1) arrows_.removeAt(index);
}

void CompositeItem::removeArrows() {
  for (Arrow *arrow : arrows_) {
    arrow->setVisible(false);

    if (arrow->start_item() != this) arrow->start_item()->removeArrow(arrow);

    if (arrow->end_item() != this) arrow->end_item()->removeArrow(arrow);

    scene()->removeItem(arrow);
    delete arrow;
  }
  arrows_.clear();
}

QVariant CompositeItem::itemChange(GraphicsItemChange change,
                                   const QVariant &value) {
  if (change == QGraphicsItem::ItemPositionChange) {
    for (Arrow *arrow : arrows_) {
      arrow->updatePosition();
    }
  }

  return value;
}
