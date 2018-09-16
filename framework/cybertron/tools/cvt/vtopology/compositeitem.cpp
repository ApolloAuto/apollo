#include "compositeitem.h"
#include <QGraphicsScene>
#include <QPainter>
#include <QPainterPath>
#include <iostream>
#include "arrow.h"

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
  foreach (Arrow *arrow, arrows_) {
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
    foreach (Arrow *arrow, arrows_) { arrow->updatePosition(); }
  }

  return value;
}
