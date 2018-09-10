#ifndef COMPOSITEITEM_H
#define COMPOSITEITEM_H

#include <QGraphicsItemGroup>
#include <QGraphicsPolygonItem>
#include <QGraphicsTextItem>
#include <QList>
#include <QVariant>
#include <QPen>

class Arrow;

class CompositeItem : public QGraphicsItemGroup {
 public:
  enum { Type = UserType + 10 };
  enum ItemType { Node, Channel };

  CompositeItem(ItemType itemType, const QString& text,
                QGraphicsItem* parent = 0);
  void SetPenColor(const QColor& c){ old_pen_.setColor(c); current_pen_.setColor(c); }
  QColor CurrentPenColor(void)const{ return current_pen_.color(); }
  QColor OldPenColor(void)const{ return old_pen_.color(); }

  virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                     QWidget* widget = 0) override;
  virtual int type() const { return Type; }

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
  virtual void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
  virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);

 private:
  ItemType item_type_;
  QPen old_pen_;
  QPen current_pen_;
  QGraphicsTextItem text_;
  QGraphicsPolygonItem shape_;
  QList<Arrow*> arrows_;
};

#endif  // COMPOSITEITEM_H
