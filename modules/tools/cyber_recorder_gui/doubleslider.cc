#include "doubleslider.h"

DoubleSlider::DoubleSlider(QWidget* parent) :
        QWidget(parent), m_min(0), m_max(100), m_leftValue(m_min), m_rightValue(m_max), m_activeHandle(None) {
    setFixedHeight(40);
}

void DoubleSlider::setRange(int min, int max) {
    m_min = min;
    m_max = max;
    if (m_max <= m_min) {
        m_max = m_min + 1;
    }
    m_leftValue = std::clamp(m_leftValue, m_min, m_max);
    m_rightValue = std::clamp(m_rightValue, m_min, m_max);
    if (m_leftValue > m_rightValue) {
        m_leftValue = m_rightValue;
    }
    update();
}

int DoubleSlider::leftValue() const {
    return m_leftValue;
}

int DoubleSlider::rightValue() const {
    return m_rightValue;
}

void DoubleSlider::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制背景
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(200, 200, 200));
    painter.drawRoundedRect(0, height() / 2 - 2, width(), 4, 2, 2);

    // 绘制活动区域
    painter.setBrush(QColor(100, 150, 250));
    painter.drawRoundedRect(leftHandlePos(), height() / 2 - 2, rightHandlePos() - leftHandlePos(), 4, 2, 2);

    // 绘制左右滑块
    drawHandle(&painter, leftHandlePos(), true);
    drawHandle(&painter, rightHandlePos(), false);
}

void DoubleSlider::mousePressEvent(QMouseEvent* event) {
    const int x = event->pos().x();
    const int left = leftHandlePos();
    const int right = rightHandlePos();

    if (std::abs(x - left) < handleSize / 2) {
        m_activeHandle = LeftHandle;
    } else if (std::abs(x - right) < handleSize / 2) {
        m_activeHandle = RightHandle;
    } else {
        m_activeHandle = None;
    }
    update();
}

void DoubleSlider::mouseMoveEvent(QMouseEvent* event) {
    if (m_activeHandle == None)
        return;

    const int x = std::clamp(event->pos().x(), 0, width());
    const int value = (x * (m_max - m_min) / width()) + m_min;

    if (m_activeHandle == LeftHandle) {
        m_leftValue = std::min(value, m_rightValue);
    } else {
        m_rightValue = std::max(value, m_leftValue);
    }

    update();
    emit rangeChanged(m_leftValue, m_rightValue);
}

void DoubleSlider::mouseReleaseEvent(QMouseEvent*) {
    m_activeHandle = None;
    update();
}

int DoubleSlider::leftHandlePos() const {
    if (m_max == m_min) {
        return 0;
    }
    return (m_leftValue - m_min) * width() / (m_max - m_min);
}

int DoubleSlider::rightHandlePos() const {
    if (m_max == m_min) {
        return width();
    }
    return (m_rightValue - m_min) * width() / (m_max - m_min);
}

void DoubleSlider::drawHandle(QPainter* painter, int x, bool isLeft) {
    painter->setPen(QPen(Qt::darkGray, 1));
    painter->setBrush(isLeft ? Qt::white : QColor(220, 220, 220));

    QRect handleRect(x - handleSize / 2, height() / 2 - handleSize / 2, handleSize, handleSize);
    painter->drawEllipse(handleRect);
}
