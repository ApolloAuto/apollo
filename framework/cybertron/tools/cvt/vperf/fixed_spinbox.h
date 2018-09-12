#ifndef FIXEDSPINBOX_H
#define FIXEDSPINBOX_H

#include <QSpinBox>

class FixedSpinBox : public QSpinBox {
 public:
  explicit FixedSpinBox(QWidget* parent = nullptr);

#if defined(__linux__)
 protected:
  void timerEvent(QTimerEvent* e);
#endif
};

#endif  // FIXEDSPINBOX_H
