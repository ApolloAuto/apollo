#include "time_line_legend_item.h"
#include <QGraphicsScene>

TimeLineLegendItem::TimeLineLegendItem(QGraphicsItem* parent)
    : QGraphicsItemGroup(parent), _legendColor(), _legendText() {
  setPos(0, 0);
  addToGroup(&_legendColor);
  addToGroup(&_legendText);

  _legendColor.setRect(0, 0, 5, 5);
  _legendColor.setPos(0, 0);
  _legendText.setPos(8, -3.0f);
}

TimeLineLegendItem::~TimeLineLegendItem() {
  removeFromGroup(&_legendColor);
  removeFromGroup(&_legendText);

  QGraphicsScene* s = scene();
  s->removeItem(&_legendColor);
  s->removeItem(&_legendText);
}

QRectF TimeLineLegendItem::boundingRect() const {
  QRectF r = _legendColor.boundingRect();
  QRectF t = _legendText.boundingRect();
  qreal tmp = r.height();
  if (tmp < t.height()) tmp = t.height();
  r.setHeight(tmp);
  tmp = r.width();
  tmp += t.width();
  tmp += 3.0f;
  r.setWidth(tmp);
  return r;
}
