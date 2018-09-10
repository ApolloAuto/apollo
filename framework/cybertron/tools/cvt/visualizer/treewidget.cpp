#include "treewidget.h"
#include <QResizeEvent>

TreeWidget::TreeWidget(QWidget *parent) : QTreeWidget(parent) {}

void TreeWidget::resizeEvent(QResizeEvent *event) {
  QTreeWidget::resizeEvent(event);
  int cw = width() / columnCount();
  for (int i = 0; i < columnCount(); ++i) {
    setColumnWidth(i, cw);
  }
}

bool TreeWidget::event(QEvent *e) {
  bool b = QTreeWidget::event(e);
  if (e->type() == QEvent::Hide) emit visibilityChanged(false);
  if (e->type() == QEvent::Show) emit visibilityChanged(true);

  return b;
}
