#ifndef TREEWIDGET_H
#define TREEWIDGET_H

#include <QTreeWidget>

class TreeWidget : public QTreeWidget {
  Q_OBJECT
 public:
  explicit TreeWidget(QWidget *parent = nullptr);
  ~TreeWidget() {}

 signals:
  void visibilityChanged(bool);

 protected:
  void resizeEvent(QResizeEvent *);
  bool event(QEvent *e);
};

#endif  // TREEWIDGET_H
