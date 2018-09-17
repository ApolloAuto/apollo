/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <QAction>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QContextMenuEvent>
#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTreeWidgetItem>

#include <iostream>

#include "fixedaspectratiowidget.h"
#include "grid.h"
#include "main_window.h"
#include "pointcloud.h"
#include "texture.h"
#include "ui_main_window.h"
#include "video_images_dialog.h"

namespace {
const char* globalTreeItemStyle = "margin-right:10px";

constexpr char* aboutMessage =
    "Cyber_Visualizer\n"
    "\n"
    "One Visualization Tool for Presenting Cybertron Channel Data\n"
    "\n"
    "F5  Play | Pause\n"
    "\n"
    "Main View PointCloud view\n"
    "All Views have right button Menu";

const char* licenseMessage =
    "Copyright 2018 The Apollo Authors. All Rights Reserved.\n"
    "\n"
    "Licensed under the Apache License, Version 2.0 (the \"License\");\n"
    "you may not use this file except in compliance with the License.\n"
    "You may obtain a copy of the License at\n"
    "\n"
    "http://www.apache.org/licenses/LICENSE-2.0\n"
    "\n"
    "Unless required by applicable law or agreed to in writing, software\n"
    "distributed under the License is distributed on an \"AS IS\" BASIS,\n"
    "WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
    "See the License for the specific language governing permissions and\n"
    "limitations under the License.\n";
}

#define MEMBER_OFFSET(StructType, Member) (size_t) & (((StructType*)0)->Member)
#define StructPtrByMemberPtr(MemberPtr, StructType, Member) \
  (StructType*)((char*)MemberPtr - MEMBER_OFFSET(StructType, Member))

struct MainWindow::VideoImgProxy {
  FixedAspectRatioWidget video_image_viewer_;
  QTreeWidgetItem root_item_;
  QTreeWidgetItem channel_name_item_;
  QTreeWidgetItem action_item_;

  QComboBox channel_name_combobox_;
  QPushButton action_item_button_;

  QMutex video_image_reader_mutex_;
  std::shared_ptr<Texture> dynamic_texture_;
  CyberChannReader<adu::common::sensor::CompressedImage>*
      video_image_channel_reader_;
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow),
      msg_dialog_(new MessageDialog),
      open_images_dialog_(nullptr),

      grid_(nullptr),
      enable_grid_checkBox_(nullptr),
      grid_root_Item_(nullptr),

      pointcloud_top_item_(nullptr),
      pointcloud_comboBox_(new QComboBox),
      pointcloud_button_(new QPushButton),
      pointcloud_channel_Reader_(nullptr),

      pointcloud_reader_mutex_(),

      pointcloud_shader_(nullptr),
      grid_shader_(nullptr) {
  ui_->setupUi(this);
  ui_->videoImageGridLayout->setContentsMargins(2, 2, 2, 2);
  ui_->videoImageWidget->setVisible(false);

  ui_->mainToolBar->addAction(ui_->actionAddGrid);
  ui_->mainToolBar->addAction(ui_->actionPointCloud);
  ui_->mainToolBar->addSeparator();
  ui_->mainToolBar->addAction(ui_->actionOpenImage);
  ui_->mainToolBar->addAction(ui_->actionOpenImages);
  ui_->mainToolBar->addAction(ui_->actionDelImage);
  ui_->mainToolBar->addSeparator();
  ui_->mainToolBar->addAction(ui_->actionPlay);
  ui_->mainToolBar->addAction(ui_->actionPause);

  pointcloud_reader_mutex_.lock();

  {
    QStringList tmp;
    tmp << "ChannelNames"
        << " ";
    all_channel_root_ = new QTreeWidgetItem(tmp);
    ui_->treeWidget->addTopLevelItem(all_channel_root_);
  }

  connect(ui_->actionAbout, SIGNAL(triggered(bool)), this, SLOT(showMessage()));
  connect(ui_->actionLicense, SIGNAL(triggered(bool)), this, SLOT(showMessage()));

  pointcloud_button_->setCheckable(true);
  pointcloud_button_->setStyleSheet(globalTreeItemStyle);
  pointcloud_comboBox_->setStyleSheet(globalTreeItemStyle);

  connect(pointcloud_button_, SIGNAL(clicked(bool)), this,
          SLOT(PlayRenderableObject(bool)));
  connect(pointcloud_comboBox_, SIGNAL(currentIndexChanged(int)), this,
          SLOT(ChangePointCloudChannel()));

  connect(ui_->treeWidget, SIGNAL(itemSelectionChanged()), this, SLOT(UpdateActions()));
  connect(ui_->treeWidget, SIGNAL(visibilityChanged(bool)), ui_->actionGlobal, SLOT(setChecked(bool)));
  connect(ui_->actionPlay, SIGNAL(triggered(bool)), this, SLOT(PlayPause()));
  connect(ui_->actionPause, SIGNAL(triggered(bool)), this, SLOT(PlayPause()));
}

MainWindow::~MainWindow() {
  for (auto item : video_image_viewer_list_) {
    item->video_image_reader_mutex_.unlock();
  }
  for (auto item : closed_video_image_viewer_list_) {
    item->video_image_reader_mutex_.unlock();
  }
  pointcloud_reader_mutex_.unlock();

  apollo::cybertron::Shutdown();

  if (pointcloud_channel_Reader_) {
    if (pointcloud_channel_Reader_->isRunning()) {
      pointcloud_channel_Reader_->quit();
    }

    if (!pointcloud_channel_Reader_->isFinished()) {
      pointcloud_channel_Reader_->wait();
    }

    delete pointcloud_channel_Reader_;
    pointcloud_channel_Reader_ = nullptr;
  }

  foreach (VideoImgProxy* item, video_image_viewer_list_) {
    if (item->video_image_channel_reader_ &&
        item->video_image_channel_reader_->isRunning()) {
      item->video_image_channel_reader_->quit();

      if (!item->video_image_channel_reader_->isFinished()) {
        item->video_image_channel_reader_->wait();
      }
      delete item->video_image_channel_reader_;
    }

    item->dynamic_texture_.reset();
    delete item;
  }

  foreach (VideoImgProxy* item, closed_video_image_viewer_list_) {
    if (item->video_image_channel_reader_ &&
        item->video_image_channel_reader_->isRunning()) {
      item->video_image_channel_reader_->quit();

      if (!item->video_image_channel_reader_->isFinished()) {
        item->video_image_channel_reader_->wait();
      }
      delete item->video_image_channel_reader_;
    }

    item->dynamic_texture_.reset();
    delete item;
  }

  delete ui_;
  delete msg_dialog_;
}

void MainWindow::calculateWH(void) {
  int count = video_image_viewer_list_.count();

  if (count > 0) {
    QSize wh;
    wh.setWidth(ui_->sceneWidget->width() - count * 2 + 2);
    wh.setHeight(50);
    wh.setWidth(wh.width() / count);
    int index = 0;

    foreach (VideoImgProxy* p, video_image_viewer_list_) {
      p->video_image_viewer_.setMinimumSize(wh);
      ui_->videoImageGridLayout->removeWidget(&p->video_image_viewer_);
      ui_->videoImageGridLayout->addWidget(&p->video_image_viewer_, index / 3,
                                           index % 3, 1, 1);
      ++index;
    }
    ui_->videoImageWidget->adjustSize();
  }
}

MainWindow::VideoImgProxy* MainWindow::AddVideoImgViewer() {
  std::shared_ptr<Texture> tex(new Texture);
  if (tex == nullptr) {
    return nullptr;
  }
  VideoImgProxy* ret = new VideoImgProxy();

  if (ret) {
    int index = video_image_viewer_list_.count();
    ret->video_image_viewer_.set_index(index);

    ret->video_image_viewer_.setStyleSheet("background-color:black;");
    ret->video_image_viewer_.setVisible(false);

    QString imgName = tr("Camera%1").arg(index);

    ret->root_item_.setHidden(true);
    ret->root_item_.setText(0, imgName);
    ret->root_item_.setText(1, "");

    ret->channel_name_item_.setText(0, "ChannelName");
    ret->action_item_.setText(0, "Action");

    ret->action_item_button_.setObjectName(tr("pushButton%1").arg(index));
    ret->action_item_button_.setText("Play");
    ret->action_item_button_.setCheckable(true);
    ret->action_item_button_.setStyleSheet(globalTreeItemStyle);

    ret->channel_name_combobox_.setObjectName(tr("comboBox%1").arg(index));
    ret->channel_name_combobox_.setStyleSheet(globalTreeItemStyle);

    ret->root_item_.addChild(&ret->channel_name_item_);
    ret->root_item_.addChild(&ret->action_item_);

    ret->dynamic_texture_ = tex;
    ret->video_image_viewer_.SetupDynamicTexture(tex);

    ret->video_image_viewer_.setSizePolicy(QSizePolicy::Preferred,
                                           QSizePolicy::Preferred);
    ret->video_image_viewer_.addAction(ui_->actionDelImage);
  }
  return ret;
}

void MainWindow::EnableGrid(bool b) { grid_->set_is_renderable(b); }

void MainWindow::ActionAddGrid(void) {
  if (grid_shader_ == nullptr) {
    grid_shader_ = RenderableObject::CreateShaderProgram(tr(":/grid.vert"),
                                                         tr(":/grid.frag"));
    if (grid_shader_ != nullptr) {
      ui_->sceneWidget->AddNewShaderProg("grid", grid_shader_);
    }
  }

  if (grid_root_Item_ == nullptr) {
    QTreeWidgetItem* colorChild = nullptr;
    QTreeWidgetItem* cellCountChild = nullptr;
    QSpinBox* spinbox = nullptr;

    grid_ = new Grid(6);
    if (grid_ == nullptr) {
      goto _ret1;
    }

    if (grid_shader_ == nullptr) {
      goto _ret2;
    }

    enable_grid_checkBox_ = new QCheckBox(ui_->treeWidget);
    if (enable_grid_checkBox_ == nullptr) {
      goto _ret2;
    }

    grid_root_Item_ = new QTreeWidgetItem(ui_->treeWidget);
    if (grid_root_Item_ == nullptr) {
      goto _ret3;
    }

    colorChild = new QTreeWidgetItem(grid_root_Item_);
    if (colorChild == nullptr) {
      goto _ret4;
    }
    grid_root_Item_->addChild(colorChild);
    colorChild->setText(0, "Color");
    colorChild->setText(1, tr("%1;%2;%3")
                               .arg(grid_->red())
                               .arg(grid_->green())
                               .arg(grid_->blue()));

    spinbox = new QSpinBox(ui_->treeWidget);
    if (spinbox == nullptr) {
      goto _ret5;
    }
    spinbox->setMinimum(1);
    spinbox->setMaximum(100);
    spinbox->setSingleStep(1);
    spinbox->setValue(grid_->CellCount());
    spinbox->setStyleSheet(globalTreeItemStyle);

    cellCountChild = new QTreeWidgetItem(grid_root_Item_);
    if (cellCountChild == nullptr) {
      goto _ret6;
    }
    grid_root_Item_->addChild(cellCountChild);
    cellCountChild->setText(0, "CellCount");

    grid_->set_shader_program(grid_shader_);
    if (!ui_->sceneWidget->AddPermanentRenderObj(grid_)) {
      goto _ret7;
    }

    enable_grid_checkBox_->setText("Enable");
    enable_grid_checkBox_->setChecked(true);
    grid_root_Item_->setText(0, "Grid");
    grid_root_Item_->setText(1, "");

    ui_->treeWidget->addTopLevelItem(grid_root_Item_);

    ui_->treeWidget->setItemWidget(grid_root_Item_, 1, enable_grid_checkBox_);
    ui_->treeWidget->setItemWidget(cellCountChild, 1, spinbox);

    connect(enable_grid_checkBox_, SIGNAL(clicked(bool)), this,
            SLOT(EnableGrid(bool)));
    connect(ui_->treeWidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
            this, SLOT(EditGridColor(QTreeWidgetItem*, int)));
    connect(spinbox, SIGNAL(valueChanged(int)), this,
            SLOT(ChangeGridCellCountBySize(int)));
    return;

  _ret7:
    delete cellCountChild;
  _ret6:
    delete spinbox;
  _ret5:
    delete colorChild;
  _ret4:
    delete grid_root_Item_;
  _ret3:
    delete enable_grid_checkBox_;
  _ret2:
    delete grid_;
  _ret1:
    QMessageBox::warning(this, tr("Error"),
                         tr("There is no enough memory for creating Grid!!!"),
                         QMessageBox::Ok);
    return;
  }
}

void MainWindow::ChangeGridCellCountBySize(int v) {
  grid_->Destroy();
  grid_->set_shader_program(grid_shader_);
  grid_->SetCellCount(v);
  grid_->set_is_renderable(true);
}

void MainWindow::EditGridColor(QTreeWidgetItem* item, int column) {
  if (column && item == grid_root_Item_->child(0)) {
    QStringList rgb = item->text(1).split(';');
    QColor color(rgb.at(0).toInt(), rgb.at(1).toInt(), rgb.at(2).toInt());

    color = QColorDialog::getColor(color, nullptr, tr("set grid color"));

    if (color.isValid()) {
      QString str =
          tr("%1;%2;%3").arg(color.red()).arg(color.green()).arg(color.blue());
      item->setText(1, str);

      grid_->set_grid_color(color);
    }
  }
}

void MainWindow::ActionOpenPointCloud(void) {
  if (pointcloud_shader_ == nullptr) {
    pointcloud_shader_ = RenderableObject::CreateShaderProgram(
        tr(":/pointcloud.vert"), tr(":/pointcloud.frag"));
    if (pointcloud_shader_ != nullptr) {
      ui_->sceneWidget->AddNewShaderProg("pointcloud", pointcloud_shader_);
    }
  }

  if (pointcloud_top_item_ == nullptr) {
    if (pointcloud_shader_ == nullptr) {
      QMessageBox::warning(this, tr("NO Shader"),
                           tr("There is no suitable shader for Pointcloud!!!"),
                           QMessageBox::Ok);
      return;
    }

    pointcloud_top_item_ = new QTreeWidgetItem(ui_->treeWidget);
    if (pointcloud_top_item_ == nullptr) {
      return;
    }

    pointcloud_top_item_->setText(0, "PointCloud2");
    pointcloud_top_item_->setText(1, "");

    QTreeWidgetItem* item = new QTreeWidgetItem(pointcloud_top_item_);
    if (item == nullptr) {
      QMessageBox::warning(this, tr("NO Enough Memory"),
                           tr("Cannot create tree item for channel name!!!"),
                           QMessageBox::Ok);
      return;
    }

    item->setText(0, "ChannelName");
    ui_->treeWidget->setItemWidget(item, 1, pointcloud_comboBox_);

    item = new QTreeWidgetItem(pointcloud_top_item_);
    if (item == nullptr) {
      QMessageBox::warning(this, tr("NO Enough Memory"),
                           tr("Cannot create tree item for channel Action!!!"),
                           QMessageBox::Ok);
      return;
    }

    item->setText(0, "Action");

    pointcloud_button_->setText("Play");
    ui_->treeWidget->setItemWidget(item, 1, pointcloud_button_);

    ui_->treeWidget->setVisible(true);
    ui_->actionGlobal->setChecked(true);
  }
}

void MainWindow::ActionOpenImages(void) {
  if (open_images_dialog_ == nullptr) {
    open_images_dialog_ = new VideoImagesDialog(this);
    if (open_images_dialog_ == nullptr) {
      QMessageBox::warning(this, tr("No Enough Memory"),
                           tr("There is no enough memory!!!"), QMessageBox::Ok);
      return;
    } else {
      connect(open_images_dialog_, SIGNAL(accepted()), this,
              SLOT(AddVideoImages()));
    }
  }

  open_images_dialog_->show();
}

void MainWindow::AddVideoImages(void) {
  int count = open_images_dialog_->count();
  while (count) {
    ActionOpenImage();
    --count;
  }
}

void MainWindow::ActionOpenImage(void) {
  VideoImgProxy* videoImgProxy;

  if (closed_video_image_viewer_list_.empty()) {
    videoImgProxy = AddVideoImgViewer();

    if (videoImgProxy == nullptr) {
      QMessageBox::warning(this, tr("No Enough Memory"),
                           tr("There is no enough memory!!!\nCannot add new "
                              "image or video channel!"),
                           QMessageBox::Ok);
      return;
    }

    videoImgProxy->video_image_viewer_.setParent(ui_->videoImageWidget);

    ui_->treeWidget->addTopLevelItem(&videoImgProxy->root_item_);
    ui_->treeWidget->setItemWidget(&videoImgProxy->channel_name_item_, 1,
                                   &videoImgProxy->channel_name_combobox_);
    ui_->treeWidget->setItemWidget(&videoImgProxy->action_item_, 1,
                                   &videoImgProxy->action_item_button_);

    for (int i = 0; i < all_channel_root_->childCount(); ++i) {
      QTreeWidgetItem* child = all_channel_root_->child(i);
      QString channel = child->text(0);
      if (channel.contains("camera")) {
        videoImgProxy->channel_name_combobox_.addItem(channel);
      }
    }
  } else {
    videoImgProxy = closed_video_image_viewer_list_.takeFirst();
    if (videoImgProxy->action_item_button_.isChecked()) {
      videoImgProxy->video_image_reader_mutex_.unlock();
    }
  }

  connect(&videoImgProxy->action_item_button_, SIGNAL(clicked(bool)), this,
          SLOT(PlayVideoImage(bool)));
  connect(&videoImgProxy->channel_name_combobox_,
          SIGNAL(currentIndexChanged(int)), this,
          SLOT(ChangeVideoImgChannel()));
  connect(&videoImgProxy->video_image_viewer_,
          SIGNAL(focusOnThis(FixedAspectRatioWidget*)), this,
          SLOT(SelectCurrentTreeItem(FixedAspectRatioWidget*)));

  video_image_viewer_list_.append(videoImgProxy);
  videoImgProxy->video_image_viewer_.setVisible(true);

  calculateWH();

  videoImgProxy->video_image_viewer_.StartOrStopUpdate(true);
  videoImgProxy->root_item_.setHidden(false);

  ui_->videoImageWidget->setVisible(true);

  ui_->treeWidget->setVisible(true);
  ui_->actionGlobal->setChecked(true);

  repaint();
}

void MainWindow::ActionDelVideoImage(void) {
  QTreeWidgetItem* topItem = ui_->treeWidget->currentItem();
  VideoImgProxy* proxy =
      StructPtrByMemberPtr(topItem, VideoImgProxy, root_item_);

  DoDeleteVideoImg(proxy);
  ui_->actionDelImage->setEnabled(false);
}

void MainWindow::CloseVideoImgViewer(bool b) {
  if (!b) {
    VideoImgViewer* dock = static_cast<VideoImgViewer*>(sender());
    DoDeleteVideoImg(
        StructPtrByMemberPtr(dock, VideoImgProxy, video_image_viewer_));
  }
}

void MainWindow::DoDeleteVideoImg(VideoImgProxy* proxy) {
  disconnect(&proxy->action_item_button_, SIGNAL(clicked(bool)), this,
             SLOT(PlayVideoImage(bool)));
  disconnect(&proxy->channel_name_combobox_, SIGNAL(currentIndexChanged(int)),
             this, SLOT(ChangeVideoImgChannel()));
  disconnect(&proxy->video_image_viewer_,
             SIGNAL(focusOnThis(FixedAspectRatioWidget*)), this,
             SLOT(SelectCurrentTreeItem(FixedAspectRatioWidget*)));

  if (proxy->action_item_button_.isChecked()) {
    proxy->video_image_reader_mutex_.lock();
  }

  proxy->video_image_viewer_.StartOrStopUpdate(false);
  proxy->root_item_.setHidden(true);
  proxy->video_image_viewer_.setVisible(false);
  ui_->videoImageGridLayout->removeWidget(&proxy->video_image_viewer_);

  video_image_viewer_list_.removeOne(proxy);
  closed_video_image_viewer_list_.append(proxy);

  if (video_image_viewer_list_.count())
    calculateWH();
  else {
    ui_->videoImageWidget->setVisible(false);
  }
}

void MainWindow::UpdateActions(void) {
  QTreeWidgetItem* item = ui_->treeWidget->currentItem();
  ui_->actionDelImage->setEnabled(false);
  if (item) {
    if (!item->parent() && item != pointcloud_top_item_ &&
        item != all_channel_root_ && item != grid_root_Item_) {
      ui_->actionDelImage->setEnabled(true);
    }
  }
}

void MainWindow::PointCloudReaderCallback(
    const std::shared_ptr<const adu::common::sensor::PointCloud>& pdata) {
  pointcloud_reader_mutex_.lock();
  pointcloud_reader_mutex_.unlock();
  PointCloud* pc = new PointCloud(pdata->point_size(), 4, pointcloud_shader_);
  if (pc) {
    if (!pc->FillData(pdata) || !ui_->sceneWidget->AddTempRenderableObj(pc)) {
      delete pc;
    }
  } else {
    std::cerr << "-----Cannot create PointCloud----------" << std::endl;
  }
}

void MainWindow::PlayRenderableObject(bool b) {
  if (b) {
    if (pointcloud_comboBox_->currentText().isEmpty()) {
      QMessageBox::warning(
          this, tr("Settup Channel Name"),
          tr("Channel Name cannot be empty!!!\nPlease Select it!"),
          QMessageBox::Ok);
      pointcloud_button_->setChecked(false);
      return;
    }

    if (!pointcloud_channel_Reader_) {
      pointcloud_channel_Reader_ =
          new CyberChannReader<adu::common::sensor::PointCloud>();

      if (!pointcloud_channel_Reader_) {
        QMessageBox::warning(this, tr("Create Cybertron Channel Reader"),
                             tr("There is no enough memory!!!\nCannot create "
                                "cybertron channel reader!"),
                             QMessageBox::Ok);
        pointcloud_button_->setChecked(false);
        return;
      }

      auto pointCallback = [this](
          const std::shared_ptr<adu::common::sensor::PointCloud>& pdata) {
        this->PointCloudReaderCallback(pdata);
      };

      if (!pointcloud_channel_Reader_->InstallCallbackAndOpen(
              pointCallback,
              pointcloud_comboBox_->currentText().toStdString())) {
        QMessageBox::warning(
            this, tr("Settup Channel Callback"),
            tr("Channel Callback cannot be installed!!!\nPlease check it!"),
            QMessageBox::Ok);
        delete pointcloud_channel_Reader_;
        pointcloud_channel_Reader_ = nullptr;

        pointcloud_button_->setChecked(false);
        return;
      }
    }

    ui_->sceneWidget->setToolTip(pointcloud_comboBox_->currentText());
    pointcloud_top_item_->setToolTip(0, pointcloud_comboBox_->currentText());
    pointcloud_comboBox_->setEnabled(false);
    pointcloud_button_->setText(tr("Stop"));

    pointcloud_reader_mutex_.unlock();
    if (!pointcloud_channel_Reader_->isRunning()) {
      pointcloud_channel_Reader_->start();
    }
  } else {
    if (pointcloud_channel_Reader_->isRunning()) {
      pointcloud_reader_mutex_.lock();
    }

    pointcloud_comboBox_->setEnabled(true);
    pointcloud_button_->setText(tr("Show"));
  }

  ui_->treeWidget->setCurrentItem(nullptr);
  ui_->treeWidget->clearFocus();
}

void MainWindow::ImageReaderCallback(
    const std::shared_ptr<const adu::common::sensor::CompressedImage>& imgData,
    VideoImgProxy* theVideoImgProxy) {
  theVideoImgProxy->video_image_reader_mutex_.lock();
  if (theVideoImgProxy->dynamic_texture_ == nullptr) {
    return;
  }
  if (imgData == nullptr) {
    return;
  }

  QImage img;
  if (img.loadFromData((const unsigned char*)(imgData->data().c_str()),
                       imgData->ByteSize())) {
    if (theVideoImgProxy->dynamic_texture_->UpdateData(img)) {
      theVideoImgProxy->video_image_viewer_.SetupDynamicTexture(
          theVideoImgProxy->dynamic_texture_);
    } else {
      std::cerr << "--------Cannot update dynamic Texture Data--------"
                << std::endl;
    }
  } else {
    std::cerr << "-----------Cannot load compressed image from data with QImage"
              << std::endl;
  }
  theVideoImgProxy->video_image_reader_mutex_.unlock();
}

void MainWindow::PlayVideoImage(bool b) {
  QPushButton* obj = static_cast<QPushButton*>(QObject::sender());
  VideoImgProxy* theVideoImg =
      StructPtrByMemberPtr(obj, VideoImgProxy, action_item_button_);
  DoPlayVideoImage(b, theVideoImg);
}

void MainWindow::DoPlayVideoImage(bool b, VideoImgProxy* theVideoImg) {
  if (b) {
    if (theVideoImg->channel_name_combobox_.currentText().isEmpty()) {
      QMessageBox::warning(
          this, tr("Settup Channel Name"),
          tr("Channel Name cannot be empty!!!\nPlease select one!"),
          QMessageBox::Ok);
      theVideoImg->action_item_button_.setChecked(false);
      return;
    }

    if (!theVideoImg->video_image_channel_reader_) {
      theVideoImg->video_image_channel_reader_ =
          new CyberChannReader<adu::common::sensor::CompressedImage>();

      if (!theVideoImg->video_image_channel_reader_) {
        QMessageBox::warning(this, tr("Create Cybertron Channel Reader"),
                             tr("There is no enough memory!!!\nCannot create "
                                "cybertron channel reader!"),
                             QMessageBox::Ok);
        return;
      }

      auto videoCallback = [this, theVideoImg](
          const std::shared_ptr<adu::common::sensor::CompressedImage>& pdata) {
        this->ImageReaderCallback(pdata, theVideoImg);
      };

      if (!theVideoImg->video_image_channel_reader_->InstallCallbackAndOpen(
              videoCallback, theVideoImg->channel_name_combobox_.currentText()
                                 .toStdString())) {
        QMessageBox::warning(
            this, tr("Settup Channel Callback"),
            tr("Channel Callback cannot be installed!!!\nPlease check it!"),
            QMessageBox::Ok);
        delete theVideoImg->video_image_channel_reader_;
        theVideoImg->video_image_channel_reader_ = nullptr;

        theVideoImg->action_item_button_.setChecked(false);
        return;
      }
    }

    theVideoImg->root_item_.setToolTip(
        0, theVideoImg->channel_name_combobox_.currentText());
    theVideoImg->video_image_viewer_.setToolTip(
        theVideoImg->channel_name_combobox_.currentText());

    theVideoImg->action_item_button_.setText("Stop");
    theVideoImg->channel_name_combobox_.setEnabled(false);

    theVideoImg->video_image_reader_mutex_.unlock();
    if (!theVideoImg->video_image_channel_reader_->isRunning())
      theVideoImg->video_image_channel_reader_->start();
  } else {
    if (theVideoImg->video_image_channel_reader_->isRunning()) {
      theVideoImg->video_image_reader_mutex_.lock();
    }

    theVideoImg->action_item_button_.setText("Play");
    theVideoImg->channel_name_combobox_.setEnabled(true);
  }
  ui_->treeWidget->setCurrentItem(nullptr);
  ui_->treeWidget->clearFocus();
}

void MainWindow::ChangePointCloudChannel() {
  if (pointcloud_channel_Reader_ != nullptr) {
    pointcloud_channel_Reader_->CloseChannel();
    pointcloud_channel_Reader_->OpenChannel(
        pointcloud_comboBox_->currentText().toStdString());
  }
}

void MainWindow::ChangeVideoImgChannel() {
  QComboBox* obj = static_cast<QComboBox*>(QObject::sender());
  VideoImgProxy* theVideoImg =
      StructPtrByMemberPtr(obj, VideoImgProxy, channel_name_combobox_);

  if (theVideoImg->video_image_channel_reader_ != nullptr) {
    theVideoImg->video_image_channel_reader_->CloseChannel();
    theVideoImg->video_image_channel_reader_->OpenChannel(
        obj->currentText().toStdString());
  }
}

void MainWindow::SelectCurrentTreeItem(FixedAspectRatioWidget* dock) {
  if (dock) {
    VideoImgProxy* theVideoImg =
        StructPtrByMemberPtr(dock, VideoImgProxy, video_image_viewer_);
    theVideoImg->root_item_.setExpanded(true);
    if (ui_->treeWidget->currentItem() == &theVideoImg->root_item_) {
      ui_->actionDelImage->setEnabled(true);
    } else {
      ui_->treeWidget->setCurrentItem(&theVideoImg->root_item_);
    }
    ui_->treeWidget->setFocus();
  }
}

void MainWindow::TopologyChanged(
    const apollo::cybertron::proto::ChangeMsg& change_msg) {
  if (::apollo::cybertron::proto::ChangeType::CHANGE_CHANNEL ==
          change_msg.change_type() &&
      ::apollo::cybertron::proto::RoleType::ROLE_WRITER ==
          change_msg.role_type() &&
      ::apollo::cybertron::proto::OperateType::OPT_JOIN ==
          change_msg.operate_type()) {
    QString str(change_msg.role_attr().channel_name().c_str());
    QStringList tmpList;
    tmpList << str;

    QTreeWidgetItem* child = new QTreeWidgetItem(tmpList);
    if (child == nullptr) {
      QMessageBox::warning(
          this, tr("Error"),
          tr("No Enough for New Channel!!!\nPlease Select it!"),
          QMessageBox::Ok);
      return;
    }
    child->setToolTip(0, str);

    bool b = true;
    for (int i = 0; i < all_channel_root_->childCount(); ++i) {
      if (str < all_channel_root_->child(i)->text(0)) {
        all_channel_root_->insertChild(i, child);
        b = false;
      }
    }

    if (b) {
      all_channel_root_->addChild(child);
    }
    ui_->treeWidget->setRootIsDecorated(true);

    if (str.contains("camera", Qt::CaseInsensitive)) {
      foreach (VideoImgProxy* item, video_image_viewer_list_) {
        item->channel_name_combobox_.addItem(str);
      }

      foreach (VideoImgProxy* item, closed_video_image_viewer_list_) {
        item->channel_name_combobox_.addItem(str);
      }
    }

    if (str.contains("pointcloud", Qt::CaseInsensitive)) {
      pointcloud_comboBox_->addItem(str);
    }
  }
}

void MainWindow::PlayPause(void) {

    QObject* obj = QObject::sender();
    bool b = true;
    if(obj == ui_->actionPause)
        b = false;

    if(pointcloud_top_item_) {
        pointcloud_button_->setChecked(b);
        PlayRenderableObject(b);
    }
    foreach(VideoImgProxy* p, video_image_viewer_list_) {
        p->action_item_button_.setChecked(b);
        DoPlayVideoImage(b, p);
    }
}

void MainWindow::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  calculateWH();
}

void MainWindow::showMessage() {
  QObject* obj = QObject::sender();

  if (obj == ui_->actionAbout) {
    msg_dialog_->setWindowTitle(tr("About"));
    msg_dialog_->setMessage(aboutMessage);
    goto showMsgLabel;
  }

  if (obj == ui_->actionLicense) {
    msg_dialog_->setWindowTitle(tr("License"));
    msg_dialog_->setMessage(licenseMessage);

  showMsgLabel:
    msg_dialog_->adjustSize();
    msg_dialog_->show();
  }
}
