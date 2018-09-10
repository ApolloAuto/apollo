#ifndef INTERACTIVEGRAPHICSVIEW_H
#define INTERACTIVEGRAPHICSVIEW_H

#include <QGraphicsView>

class InteractiveGraphicsView : public QGraphicsView {
  Q_OBJECT
 public:
  InteractiveGraphicsView(QWidget *parent = 0);
  InteractiveGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);
  ~InteractiveGraphicsView() {}

  void zoom_in();
  void zoom_out();
  void zoom(float scaleValue);
  void translate(QPointF deltaVal);

 protected:
  void keyPressEvent(QKeyEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

 private:
  void init(void);

  bool mouse_left_button_pressed_;
  QPoint last_mouse_pos_;
  double scaleVal_;
};

#endif  // INTERACTIVEGRAPHICSVIEW_H
