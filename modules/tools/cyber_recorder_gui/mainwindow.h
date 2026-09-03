#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <cmath>
#include <ctime>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <QtCore/QTimer>
#include <QtCore/QThread>
#include <QtWidgets/QMainWindow>
#include "modules/tools/cyber_recorder_gui/player/player.h"
#include "modules/tools/cyber_recorder_gui/record_playbar.h"
#include "doubleslider.h"

QT_BEGIN_NAMESPACE
using namespace apollo::goodman::record;

// 删除bazel-out/k8-opt/bin/modules/tools/cyber_recorder_gui
namespace Ui {

class MainWindow;
}  // namespace Ui
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void PlayProcessTick();

    void on_pushButton_3_clicked();
    void on_pushButton_5_clicked();

    void HandlePlaybarPressed();
    void HandlePlaybarJump();
    void HandlePlaybarReleased();
    void HandlePlaybarActionTriggered(int action);

private:
    bool HasInitializedPlayer() const;
    bool LoadPlayerFromFiles();
    void SetStatus(const QString &status);
    void UpdateRangeLabel();
    void UpdateUiState();
    void GetTimelineRange(int min, int max);
    Ui::MainWindow *ui;
    RecordPlaybar *playbar;
    QTimer *play_process_timer;
    PlayParam play_param;
    // QThread play_process_thread;
    std::vector<std::string> opt_file_vec;
    std::vector<std::string> opt_output_vec;
    std::vector<std::string> opt_white_channels;
    std::vector<std::string> opt_black_channels;
    bool is_play_ = false;
    bool opt_all = false;
    bool opt_loop = false;
    float opt_rate = 1.0f;
    uint64_t opt_begin = 0;
    uint64_t opt_end = std::numeric_limits<uint64_t>::max();
    double opt_start = 0;
    uint64_t opt_delay = 0;
    uint32_t opt_preload = 3;
    QString double_to_qstring(double dValue, int iPos = 1) {
        QString qstrValue;
        int iValue = dValue * std::pow(10, iPos);
        qstrValue = QString::number(iValue);
        int insertIndex = qstrValue.length() - iPos;
        if (0 == insertIndex) {
            qstrValue.insert(insertIndex, '.');
            qstrValue.insert(insertIndex, '0');
        } else {
            qstrValue.insert(insertIndex, '.');
        }

        return qstrValue;
    }
    DoubleSlider *slider_;
    int max_range_ = 100;
    void StopAndReset();
    std::string nsToDateTime(long long ns_timestamp);
    std::shared_ptr<Player> player_;
    QStringList getRecordFiles(const QString &directoryPath);
};

#endif  // MAINWINDOW_H
