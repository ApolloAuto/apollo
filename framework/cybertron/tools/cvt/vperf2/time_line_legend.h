#ifndef TIMELINELEGEND_H
#define TIMELINELEGEND_H

#include <QGraphicsItemGroup>
#include <QList>

class TimeLineLegendItem;

class TimeLineLegend : public QGraphicsItemGroup {
 public:
  enum { Type = QGraphicsItemGroup::UserType + QGraphicsItemGroup::Type + 3 };

  explicit TimeLineLegend(QGraphicsItem* parent = nullptr);
  ~TimeLineLegend();

  int type() const { return Type; }

  bool setLegendItemsCount(int count);

  TimeLineLegendItem* itemByIndex(int index) {
    if (index > -1 && index < _items.count()) {
      return _items.at(index);
    }
    return nullptr;
  }

  qreal legendWidth(void);

  void adjustItemsPos(void);

 private:
  QList<TimeLineLegendItem*> _items;
};

#endif  // TIMELINELEGEND_H
