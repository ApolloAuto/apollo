#include "fixed_spinbox.h"
#include <QTimerEvent>

FixedSpinBox::FixedSpinBox(QWidget *parent) : QSpinBox(parent) {}

#if defined(__linux__)
void FixedSpinBox::timerEvent(QTimerEvent *e) { e->accept(); }
#endif
