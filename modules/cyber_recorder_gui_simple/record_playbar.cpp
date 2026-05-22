#include "record_playbar.h"

// 构造函数的实现
RecordPlaybar::RecordPlaybar(Qt::Orientation orientation, QWidget *parent) : QSlider(orientation, parent) {
    connect(this, &QSlider::sliderPressed, this, &RecordPlaybar::onSliderPressed);
    // 连接 sliderReleased 信号到槽函数
    connect(this, &QSlider::sliderReleased, this, &RecordPlaybar::onSliderReleased);
}

// 鼠标按下事件处理函数的实现
void RecordPlaybar::mousePressEvent(QMouseEvent *event) {
    if (!is_jump_) {
        return;
    }
    // 调用基类的鼠标按下事件处理函数，确保默认行为
    QSlider::mousePressEvent(event);

    // 获取滑块的最小值和最大值
    int minVal = minimum();
    int maxVal = maximum();

    // 根据滑块的方向处理点击事件
    if (orientation() == Qt::Horizontal) {
        // 计算点击位置在滑块总长度中的比例
        double clickRatio = static_cast<double>(event->pos().x()) / width();
        // 根据比例计算对应的滑块值
        newVal_ = minVal + static_cast<int>(clickRatio * (maxVal - minVal));
        emit jumpSignal();
        // 设置滑块的值
        setValue(newVal_);
    } else if (orientation() == Qt::Vertical) {
        // 计算点击位置在滑块总长度中的比例
        double clickRatio = static_cast<double>(event->pos().y()) / height();
        // 根据比例计算对应的滑块值
        newVal_ = minVal + static_cast<int>((1 - clickRatio) * (maxVal - minVal));
        // 设置滑块的值
        emit jumpSignal();
        setValue(newVal_);
    }
}
