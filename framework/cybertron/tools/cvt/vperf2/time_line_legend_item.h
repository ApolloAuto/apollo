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

  explicit TimeLineLegendItem(QGraphicsItem* parent = nullptr);
  ~TimeLineLegendItem();

  int type() const { return Type; }
  QRectF boundingRect() const override;

  void setLegendColor(const QColor& c) {
    _legendColor.setBrush(QBrush(c));
    _legendColor.setPen(QPen(c));
  }

  void setLegendText(const QString& text) { _legendText.setPlainText(text); }

  void setLegendColorText(const QColor& c, const QString& text) {
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
