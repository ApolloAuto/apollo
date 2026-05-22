#ifndef RECORDPLAYBAR_H
#define RECORDPLAYBAR_H

#include <QtWidgets/QSlider>
#include <QMouseEvent>
#include <QtCore/QDebug>

class RecordPlaybar : public QSlider {
    Q_OBJECT
public:
    // explicit RecordPlaybar(QWidget *parent = nullptr);
    // 构造函数，接收滑块方向和父窗口指针
    explicit RecordPlaybar(Qt::Orientation orientation, QWidget *parent = nullptr);
    int GetNewVal() {
        return this->newVal_;
    }
    bool is_jump() {
        return this->is_jump_;
    }

protected:
    // 重写鼠标按下事件处理函数
    void mousePressEvent(QMouseEvent *event) override;
private slots:
    void onSliderPressed() {
        qDebug() << "onSliderPressed";
        is_jump_ = false;
    }

    void onSliderReleased() {
        qDebug() << "onSliderReleased";

        is_jump_ = true;
    }

private:
    int newVal_ = 0;
    bool is_jump_ = true;

signals:
    void jumpSignal();
};

#endif  // RECORDPLAYBAR_H
