#ifndef DOUBLESLIDER_H
#define DOUBLESLIDER_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <algorithm>
#include <QVBoxLayout>
#include <iostream>
#include <QDebug>

class DoubleSlider : public QWidget {
    Q_OBJECT
public:
    explicit DoubleSlider(QWidget* parent = nullptr);

    void setRange(int min, int max);
    int leftValue() const;
    int rightValue() const;

signals:
    void rangeChanged(int min, int max);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent*) override;

private:
    enum Handle { None, LeftHandle, RightHandle };

    int m_min;
    int m_max;
    int m_leftValue;
    int m_rightValue;
    Handle m_activeHandle;
    const int handleSize = 16;

    int leftHandlePos() const;
    int rightHandlePos() const;
    void drawHandle(QPainter* painter, int x, bool isLeft);
};

#endif  // DOUBLESLIDER_H