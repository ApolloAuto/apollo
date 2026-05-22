#include <getopt.h>
#include <csignal>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/time_conversion.h"
#include "cyber/init.h"
#include "modules/cyber_recorder_gui_simple/info.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtCore/QDebug>
#include <QtWidgets/QFileDialog>
#include <QDir>

#include <QString>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    Qt::WindowFlags m_flags = windowFlags();
    this->setWindowFlags(m_flags | Qt::WindowStaysOnTopHint);  // this可以省略

    playbar = new RecordPlaybar(Qt::Horizontal, this);
    playbar->setRange(0, max_range_);
    // 将 Playbar 添加到布局中
    ui->horizontalLayout_2->addWidget(playbar);
    slider_ = new DoubleSlider(this);

    // slider = new DoubleSlider(this);
    slider_->setRange(0, max_range_);
    ui->horizontalLayout_2->addWidget(slider_);

    play_process_timer = new QTimer(this);
    play_process_timer->setInterval(100);
    connect(play_process_timer, &QTimer::timeout, this, &MainWindow::PlayProcessTick);

    connect(playbar, &QSlider::sliderReleased, this, &MainWindow::HandlePlaybarReleased);
    connect(playbar, &QSlider::sliderPressed, this, &MainWindow::HandlePlaybarPressed);
    connect(playbar, &RecordPlaybar::jumpSignal, this, &MainWindow::HandlePlaybarJump);

    connect(playbar, &QSlider::actionTriggered, this, &MainWindow::HandlePlaybarActionTriggered);

    connect(slider_, &DoubleSlider::rangeChanged, [this](int min, int max) {
        if (player_ != nullptr) {
            qDebug() << "Selected range:" << min << "-" << max;
            UpdateRangeLabel();
        }
    });
    UpdateUiState();
}
void MainWindow::GetTimelineRange(int min, int max) {
    qDebug() << "Selected range:" << min << "-" << max;
}
MainWindow::~MainWindow() {
    play_process_timer->stop();
    if (player_ != nullptr) {
        player_->Stop();
    }
    delete ui;
}
bool MainWindow::HasInitializedPlayer() const {
    return player_ != nullptr && player_->is_initialized();
}

void MainWindow::SetStatus(const QString& status) {
    ui->statusLabel->setText(status);
}

void MainWindow::UpdateRangeLabel() {
    if (!HasInitializedPlayer()) {
        ui->rangeLabel->setText("Range: full record");
        return;
    }
    uint64_t total_ns = player_->get_parm().end_time_ns - player_->get_parm().begin_time_ns;
    double start_percent = slider_->leftValue() / (double)max_range_;
    double end_percent = slider_->rightValue() / (double)max_range_;
    uint64_t range_start = total_ns * start_percent + player_->get_parm().begin_time_ns;
    uint64_t range_end = total_ns * end_percent + player_->get_parm().begin_time_ns;
    ui->rangeLabel->setText(
            QString("Range: %1 - %2")
                    .arg(QString::fromStdString(nsToDateTime(range_start)))
                    .arg(QString::fromStdString(nsToDateTime(range_end))));
}

void MainWindow::UpdateUiState() {
    const bool has_player = HasInitializedPlayer();
    ui->pushButton_2->setEnabled(has_player);
    ui->pushButton_3->setEnabled(has_player);
    if (!has_player || !is_play_) {
        ui->pushButton_2->setText("Play");
    } else if (player_->is_paused()) {
        ui->pushButton_2->setText("Resume");
    } else {
        ui->pushButton_2->setText("Pause");
    }
}

bool MainWindow::LoadPlayerFromFiles() {
    if (opt_file_vec.empty()) {
        ui->textEdit->setText("No record files found.");
        SetStatus("No record files found");
        UpdateRangeLabel();
        UpdateUiState();
        return false;
    }
    if (is_play_) {
        StopAndReset();
    } else if (player_ != nullptr) {
        player_->Stop();
    }

    play_param.is_play_all_channels = opt_all || opt_white_channels.empty();
    play_param.is_loop_playback = opt_loop;
    play_param.play_rate = opt_rate;
    play_param.begin_time_ns = opt_begin;
    play_param.end_time_ns = opt_end;
    play_param.start_time_s = opt_start;
    play_param.delay_time_s = opt_delay;
    play_param.preload_time_s = opt_preload;
    play_param.files_to_play.clear();
    play_param.files_to_play.insert(opt_file_vec.begin(), opt_file_vec.end());

    play_param.black_channels.clear();
    play_param.black_channels.insert(opt_black_channels.begin(), opt_black_channels.end());
    play_param.channels_to_play.clear();
    play_param.channels_to_play.insert(opt_white_channels.begin(), opt_white_channels.end());

    SetStatus("Loading record files");
    UpdateUiState();
    player_ = std::make_shared<Player>(play_param);
    if (!player_->Init()) {
        ui->textEdit->setText("Record file init failed.");
        SetStatus("Record init failed");
        qDebug() << "record file init failed!";
        player_.reset();
        UpdateRangeLabel();
        UpdateUiState();
        return false;
    }
    SetStatus(QString("%1 record file(s) loaded").arg(opt_file_vec.size()));
    UpdateRangeLabel();
    UpdateUiState();
    return true;
}

void MainWindow::PlayProcessTick() {
    if (!HasInitializedPlayer() || player_->total_progress_time_s() <= 0) {
        play_process_timer->stop();
        UpdateUiState();
        return;
    }
    playbar->setValue(player_->progress_time_s() / player_->total_progress_time_s() * max_range_);
    ui->label->setText(
            double_to_qstring(player_->progress_time_s(), 1) + "/"
            + double_to_qstring(player_->total_progress_time_s(), 1));
    if (player_->progress_time_s() >= player_->total_progress_time_s()) {
        StopAndReset();
    }
}

QStringList MainWindow::getRecordFiles(const QString& directoryPath) {
    QDir directory(directoryPath);
    QStringList filters;
    filters << "*.record" << "*.record.*";
    directory.setNameFilters(filters);
    directory.setFilter(QDir::Files | QDir::NoDotAndDotDot);
    directory.setSorting(QDir::Name);
    return directory.entryList();
}

void MainWindow::on_pushButton_5_clicked() {
    QString selectedDirectory = QFileDialog::getExistingDirectory(
            nullptr,
            tr("Select Directory"),
            "/apollo_workplace/data/",
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!selectedDirectory.isEmpty()) {
        ui->lineEdit->setText(selectedDirectory);
        QStringList files = getRecordFiles(selectedDirectory);
        opt_file_vec.clear();
        for (const auto& file_name : files) {
            opt_file_vec.emplace_back((selectedDirectory + "/" + file_name).toStdString());
        }

        if (!files.empty()) {
            const QString first_record_file = selectedDirectory + "/" + files.first();
            Info info;
            std::string info_text;
            if (info.Display(first_record_file.toUtf8().constData(), &info_text)) {
                ui->textEdit->setText(
                        QString("Loaded %1 record file(s).\n\nPreviewing first file:\n%2")
                                .arg(files.size())
                                .arg(QString::fromStdString(info_text)));
            }
        }

        LoadPlayerFromFiles();

    } else {
        // 如果用户取消了选择，输出相应信息
        SetStatus("Directory selection canceled");
        qDebug() << "你取消了文件选择。";
    }
    return;
}
void MainWindow::on_pushButton_clicked() {
    QString dialog_title = "select record file";

    QString default_dir = QCoreApplication::applicationDirPath();
    QString filter = "ALL (*.*);;文本文件 (*.record.*)";

    QString selected_file_path = QFileDialog::getOpenFileName(nullptr, dialog_title, default_dir, filter);

    if (!selected_file_path.isEmpty()) {
        opt_file_vec.clear();
        ui->lineEdit->setText(selected_file_path);
        Info info;
        std::string info_text;
        bool info_result = info.Display(selected_file_path.toUtf8().constData(), &info_text);
        if (!info_result) {
            ui->textEdit->setText("Failed to read record file info.");
            SetStatus("Failed to read record info");
            UpdateUiState();
            return;
        }
        ui->textEdit->setText(QString::fromStdString(info_text));
        /////
        opt_file_vec.emplace_back(ui->lineEdit->text().toStdString());
        LoadPlayerFromFiles();

    } else {
        // 如果用户取消了选择，输出相应信息
        SetStatus("File selection canceled");
        qDebug() << "你取消了文件选择。";
    }
}

void MainWindow::on_pushButton_2_clicked() {
    if (HasInitializedPlayer()) {
            if (!is_play_) {
                is_play_ = player_->Start();
                if (is_play_) {
                    play_process_timer->start();
                    SetStatus("Playing");
                }
            } else {
                player_->set_is_paused();
                if (player_->is_paused()) {
                    SetStatus("Paused");
                } else {
                    SetStatus("Playing");
                }
            }
            UpdateUiState();
    }
}
void MainWindow::StopAndReset() {
    play_process_timer->stop();
    if (!HasInitializedPlayer()) {
        is_play_ = false;
        playbar->setValue(0);
        ui->label->setText("0.0/0.0");
        SetStatus("Ready");
        UpdateRangeLabel();
        UpdateUiState();
        return;
    }
    if (player_->Reset()) {
        is_play_ = false;
        playbar->setValue(0);
        ui->label->setText("0.0/0.0");
        SetStatus("Stopped");
        UpdateRangeLabel();
        UpdateUiState();
        return;
    }
    is_play_ = false;
    SetStatus("Stopped");
    UpdateUiState();
}
void MainWindow::on_pushButton_3_clicked() {
    StopAndReset();
}

void MainWindow::HandlePlaybarPressed() {
    qDebug() << "is_play_=" << is_play_;
    if (is_play_) {
        play_process_timer->stop();
    }
}

void MainWindow::HandlePlaybarReleased() {
    qDebug() << "is_play_=" << is_play_;
    if (is_play_) {
        double start_time = playbar->value() / (double)max_range_ * player_->total_progress_time_s();
        qDebug() << "start_time";
        play_process_timer->stop();
        player_->ResetProcessTime(start_time);
        play_process_timer->start();
    }
}
void MainWindow::HandlePlaybarJump() {
    qDebug() << "is_play_=" << is_play_;

    if (is_play_) {
        if (playbar->is_jump()) {
            play_process_timer->stop();
            double start_time = playbar->GetNewVal() / (double)max_range_ * player_->total_progress_time_s();
            player_->ResetProcessTime(start_time);
            play_process_timer->start();
        }
    }
}

std::string MainWindow::nsToDateTime(long long ns_timestamp) {
    // 将纳秒转换为秒
    auto sec_timestamp = std::chrono::seconds(ns_timestamp / 1000000000);
    auto remainder_ns = ns_timestamp % 1000000000;

    // 创建时间点
    auto time_point = std::chrono::time_point<std::chrono::system_clock>(sec_timestamp);

    // 将时间点转换为系统时间
    std::time_t current_time = std::chrono::system_clock::to_time_t(time_point);

    // 转换为 tm 结构
    std::tm* time_info = std::localtime(&current_time);

    // 格式化输出日期时间
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info);
    return std::string(buffer);
}
void MainWindow::HandlePlaybarActionTriggered(int action) {
    // if (action == 3 || action == 4) {
    //     int new_pos = playbar->GetNewVal();
    //     qDebug() << "action=" << action << "      " << new_pos;
    // }
}
