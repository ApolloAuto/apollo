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

#ifndef TOOLS_CVT_VTOPOLOGY_COMPOSITEITEM_H_
#define TOOLS_CVT_VTOPOLOGY_COMPOSITEITEM_H_

#include <QGraphicsItemGroup>
#include <QGraphicsPolygonItem>
#include <QGraphicsTextItem>
#include <QList>
#include <QPen>
#include <QVariant>

class Arrow;

class CompositeItem : public QGraphicsItemGroup {
 public:
  enum { Type = UserType + 10 };
  enum ItemType { Node, Channel };

  CompositeItem(ItemType itemType, const QString& text,
                QGraphicsItem* parent = 0);
  void SetPenColor(const QColor& c) {
    old_pen_.setColor(c);
    current_pen_.setColor(c);
  }
  QColor CurrentPenColor(void) const { return current_pen_.color(); }
  QColor OldPenColor(void) const { return old_pen_.color(); }

  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget = 0) override;
  int type() const override { return Type; }

  const QString text(void) const { return text_.toPlainText(); }

  QPolygonF polygon() const;

  void AddArrow(Arrow* arrow) {
    if (arrow) {
      arrows_.append(arrow);
    }
  }
  void removeArrow(Arrow* arrow);
  void removeArrows(void);

 protected:
  QVariant itemChange(GraphicsItemChange change,
                      const QVariant& value) override;
  virtual void hoverEnterEvent(QGraphicsSceneHoverEvent* event);
  virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent* event);

 private:
  ItemType item_type_;
  QPen old_pen_;
  QPen current_pen_;
  QGraphicsTextItem text_;
  QGraphicsPolygonItem shape_;
  QList<Arrow*> arrows_;
};

#endif  // COMPOSITEITEM_H
