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

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QColorDialog>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>

#include "modules/tools/visualizer/fixedaspectratiowidget.h"
#include "modules/tools/visualizer/grid.h"
#include "modules/tools/visualizer/main_window.h"
#include "modules/tools/visualizer/pointcloud.h"
#include "modules/tools/visualizer/radarpoints.h"
#include "modules/tools/visualizer/ui_main_window.h"
#include "modules/tools/visualizer/video_images_dialog.h"

namespace {
const char* globalTreeItemStyle = "margin-right:10px";

const char* aboutMessage =
    "Cyber_Visualizer\n"
    "\n"
    "One Visualization Tool for Presenting Cyber Channel Data\n"
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

const char* pcTempObjGroupName = "pointcloud";
const char* pcVertexPath = "/apollo/modules/tools/visualizer/shaders/pointcloud.vert";
const char* pcFragPath = "/apollo/modules/tools/visualizer/shaders/grid_pointcloud.frag";
const char* gridVertexPath = "/apollo/modules/tools/visualizer/shaders/grid.vert";
const char* gridFragPath = "/apollo/modules/tools/visualizer/shaders/grid_pointcloud.frag";
const char* radarVertexPath = "/apollo/modules/tools/visualizer/shaders/radarpoints.vert";
const char* radarFragPath = "/apollo/modules/tools/visualizer/shaders/radarpoints.frag";

const std::string CompressedImageType("apollo.drivers.CompressedImage");

}  // namespace

#define MEMBER_OFFSET(StructType, Member)                                    \
  (size_t)(                                                                  \
      reinterpret_cast<char*>(&(reinterpret_cast<StructType*>(1)->Member)) - \
      1)
#define StructPtrByMemberPtr(MemberPtr, StructType, Member)          \
  reinterpret_cast<StructType*>(reinterpret_cast<char*>(MemberPtr) - \
                                MEMBER_OFFSET(StructType, Member))

struct MainWindow::VideoImgProxy {
  FixedAspectRatioWidget video_image_viewer_;
  QTreeWidgetItem root_item_;
  QTreeWidgetItem channel_name_item_;
  QTreeWidgetItem action_item_;

  QComboBox channel_name_combobox_;
  QPushButton action_item_button_;

  QMutex reader_mutex_;
  std::shared_ptr<Texture> dynamic_texture_;
  bool isCompressedImage_;
  union {
    CyberChannReader<apollo::drivers::Image>* image_reader_;
    CyberChannReader<apollo::drivers::CompressedImage>*
        compressed_image_reader_;
  };

  VideoImgProxy()
      : video_image_viewer_(),
        root_item_(),
        channel_name_item_(),
        action_item_(),
        channel_name_combobox_(),
        action_item_button_(),
        reader_mutex_(),
        dynamic_texture_(),
        isCompressedImage_(false),
        image_reader_(nullptr) {}

  void deleteReader(void) {
    if (image_reader_) {
      if (isCompressedImage_) {
        delete compressed_image_reader_;
      } else {
        delete image_reader_;
      }
      image_reader_ = nullptr;
      isCompressedImage_ = false;
    }
  }

  void CloseChannel(void) {
    if (isCompressedImage_) {
      compressed_image_reader_->CloseChannel();
    } else {
      image_reader_->CloseChannel();
    }
  }

  ~VideoImgProxy(void) {
    dynamic_texture_.reset();
    deleteReader();
  }
};

struct MainWindow::RadarData {
  QTreeWidgetItem root_item_;
  QTreeWidgetItem channel_name_item_;
  QTreeWidgetItem action_item_;

  QComboBox channel_name_combobox_;
  QPushButton action_item_button_;
  QCheckBox enable_checkBox_;

  QMutex reader_mutex_;

  CyberChannReader<apollo::drivers::RadarObstacles>* channel_reader_;

  RadarData(void)
      : root_item_(),
        channel_name_item_(),
        action_item_(),
        channel_name_combobox_(),
        action_item_button_(),
        enable_checkBox_(),
        reader_mutex_(),
        channel_reader_(nullptr) {}

  ~RadarData(void) {
    if (channel_reader_) {
      delete channel_reader_;
      channel_reader_ = nullptr;
    }
  }
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow),
      msg_dialog_(new MessageDialog),
      open_images_dialog_(nullptr),

      grid_(nullptr),
      enable_grid_checkBox_(nullptr),
      grid_root_item_(nullptr),

      pointcloud_top_item_(nullptr),
      pointcloud_comboBox_(new QComboBox),
      pointcloud_button_(new QPushButton),
      pointcloud_channel_Reader_(nullptr),

      pointcloud_reader_mutex_(),

      pointcloud_shader_(nullptr),
      grid_shader_(nullptr),
      radar_points_shader_(nullptr),
      video_image_viewer_list_(),
      closed_video_image_viewer_list_(),
      radarData_list_(),
      closed_radarData_list_(),
      _channelName2TypeMap() {
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
  connect(ui_->actionLicense, SIGNAL(triggered(bool)), this,
          SLOT(showMessage()));

  pointcloud_button_->setCheckable(true);
  pointcloud_button_->setStyleSheet(globalTreeItemStyle);
  pointcloud_comboBox_->setStyleSheet(globalTreeItemStyle);

  connect(pointcloud_button_, SIGNAL(clicked(bool)), this,
          SLOT(PlayRenderableObject(bool)));
  connect(pointcloud_comboBox_, SIGNAL(currentIndexChanged(int)), this,
          SLOT(ChangePointCloudChannel()));

  connect(ui_->treeWidget, SIGNAL(itemSelectionChanged()), this,
          SLOT(UpdateActions()));
  connect(ui_->treeWidget, SIGNAL(visibilityChanged(bool)), ui_->actionGlobal,
          SLOT(setChecked(bool)));
  connect(ui_->actionPlay, SIGNAL(triggered(bool)), this, SLOT(PlayPause()));
  connect(ui_->actionPause, SIGNAL(triggered(bool)), this, SLOT(PlayPause()));
}

MainWindow::~MainWindow() {
  for (VideoImgProxy* item : video_image_viewer_list_) {
    item->reader_mutex_.unlock();
  }
  for (VideoImgProxy* item : closed_video_image_viewer_list_) {
    item->reader_mutex_.unlock();
  }
  for (RadarData* item : radarData_list_) {
    item->reader_mutex_.unlock();
  }
  for (RadarData* item : closed_radarData_list_) {
    item->reader_mutex_.unlock();
  }

  pointcloud_reader_mutex_.unlock();

  if (pointcloud_channel_Reader_) {
    delete pointcloud_channel_Reader_;
    pointcloud_channel_Reader_ = nullptr;
  }

  for (VideoImgProxy* item : video_image_viewer_list_) {
    delete item;
  }

  for (VideoImgProxy* item : closed_video_image_viewer_list_) {
    delete item;
  }

  for (RadarData* item : radarData_list_) {
    delete item;
  }

  for (RadarData* item : closed_radarData_list_) {
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

    for (VideoImgProxy* p : video_image_viewer_list_) {
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
    ret->reader_mutex_.lock();
  }
  return ret;
}

MainWindow::RadarData* MainWindow::createRadarData(void) {
  RadarData* ret = new RadarData();

  if (ret) {
    int index = radarData_list_.count();
    QString radarName = tr("Radar%1").arg(index);

    ret->root_item_.setHidden(true);
    ret->root_item_.setText(0, radarName);

    ret->channel_name_item_.setText(0, "ChannelName");
    ret->action_item_.setText(0, "Action");
    ret->enable_checkBox_.setChecked(true);

    ret->action_item_button_.setText("Play");
    ret->action_item_button_.setCheckable(true);
    ret->action_item_button_.setStyleSheet(globalTreeItemStyle);

    ret->channel_name_combobox_.setObjectName(tr("comboBox%1").arg(index));
    ret->channel_name_combobox_.setStyleSheet(globalTreeItemStyle);

    ret->root_item_.addChild(&ret->channel_name_item_);
    ret->root_item_.addChild(&ret->action_item_);
    ret->reader_mutex_.lock();
  }
  return ret;
}

void MainWindow::EnableGrid(bool b) { grid_->set_is_renderable(b); }

void MainWindow::ActionAddGrid(void) {
  if (grid_shader_ == nullptr) {
    grid_shader_ = RenderableObject::CreateShaderProgram(tr(gridVertexPath),
                                                         tr(gridFragPath));
    if (grid_shader_ != nullptr) {
      ui_->sceneWidget->AddNewShaderProg("grid", grid_shader_);
    }
  }

  if (grid_root_item_ == nullptr) {
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

    grid_root_item_ = new QTreeWidgetItem(ui_->treeWidget);
    if (grid_root_item_ == nullptr) {
      goto _ret3;
    }

    colorChild = new QTreeWidgetItem(grid_root_item_);
    if (colorChild == nullptr) {
      goto _ret4;
    }
    grid_root_item_->addChild(colorChild);
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

    cellCountChild = new QTreeWidgetItem(grid_root_item_);
    if (cellCountChild == nullptr) {
      goto _ret6;
    }
    grid_root_item_->addChild(cellCountChild);
    cellCountChild->setText(0, "CellCount");

    grid_->set_shader_program(grid_shader_);
    if (!ui_->sceneWidget->AddPermanentRenderObj(grid_)) {
      goto _ret7;
    }

    enable_grid_checkBox_->setText("Enable");
    enable_grid_checkBox_->setChecked(true);
    grid_root_item_->setText(0, "Grid");
    grid_root_item_->setText(1, "");

    ui_->treeWidget->addTopLevelItem(grid_root_item_);

    ui_->treeWidget->setItemWidget(grid_root_item_, 1, enable_grid_checkBox_);
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
    delete grid_root_item_;
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
  if (column && item != nullptr && grid_root_item_ != nullptr &&
      item == grid_root_item_->child(0)) {
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

void MainWindow::EnableRadarPoints(bool b) {
  QCheckBox* obj = static_cast<QCheckBox*>(sender());
  RadarData* r = StructPtrByMemberPtr(obj, RadarData, enable_checkBox_);
  ui_->sceneWidget->setTempObjGroupEnabled(r->root_item_.text(0).toStdString(),
                                           b);
}

void MainWindow::ActionOpenRadarChannel(void) {
  if (radar_points_shader_ == nullptr) {
    radar_points_shader_ = RenderableObject::CreateShaderProgram(
        tr(radarVertexPath), tr(radarFragPath));
    if (radar_points_shader_ != nullptr) {
      ui_->sceneWidget->AddNewShaderProg("radarpoints", radar_points_shader_);
    } else {
      QMessageBox::warning(
          this, tr("NO Shader"),
          tr("There is no suitable shader for Radar Points!!!"),
          QMessageBox::Ok);
      return;
    }
  }

  RadarData* radarProxy;

  if (closed_radarData_list_.empty()) {
    radarProxy = createRadarData();

    if (radarProxy == nullptr) {
      QMessageBox::warning(this, tr("No Enough Memory"),
                           tr("There is no enough memory!!!\nCannot add new "
                              "image or video channel!"),
                           QMessageBox::Ok);
      return;
    }

    ui_->treeWidget->addTopLevelItem(&radarProxy->root_item_);
    ui_->treeWidget->setItemWidget(&radarProxy->root_item_, 1,
                                   &radarProxy->enable_checkBox_);

    ui_->treeWidget->setItemWidget(&radarProxy->channel_name_item_, 1,
                                   &radarProxy->channel_name_combobox_);
    ui_->treeWidget->setItemWidget(&radarProxy->action_item_, 1,
                                   &radarProxy->action_item_button_);

    for (int i = 0; i < all_channel_root_->childCount(); ++i) {
      QTreeWidgetItem* child = all_channel_root_->child(i);
      QString channel = child->text(0);
      if (channel.contains("radar")) {
        radarProxy->channel_name_combobox_.addItem(channel);
      }
    }
  } else {
    radarProxy = closed_radarData_list_.takeFirst();
  }

  connect(&radarProxy->action_item_button_, SIGNAL(clicked(bool)), this,
          SLOT(openRadarChannel(bool)));
  connect(&radarProxy->enable_checkBox_, SIGNAL(clicked(bool)), this,
          SLOT(EnableRadarPoints(bool)));
  connect(&radarProxy->channel_name_combobox_, SIGNAL(currentIndexChanged(int)),
          this, SLOT(ChangeRadarChannel()));

  radarData_list_.append(radarProxy);

  radarProxy->root_item_.setHidden(false);

  ui_->treeWidget->setVisible(true);
  ui_->actionGlobal->setChecked(true);
}

void MainWindow::openRadarChannel(bool b) {
  QPushButton* obj = static_cast<QPushButton*>(QObject::sender());
  RadarData* theVideoImg =
      StructPtrByMemberPtr(obj, RadarData, action_item_button_);
  DoOpenRadarChannel(b, theVideoImg);
}

void MainWindow::DoOpenRadarChannel(bool b, RadarData* radarProxy) {
  if (b) {
    if (radarProxy->channel_name_combobox_.currentText().isEmpty()) {
      QMessageBox::warning(
          this, tr("Setup Channel Name"),
          tr("Channel Name cannot be empty!!!\nPlease select one!"),
          QMessageBox::Ok);
      radarProxy->action_item_button_.setChecked(false);
      return;
    }

    if (!radarProxy->channel_reader_) {
      radarProxy->channel_reader_ =
          new CyberChannReader<apollo::drivers::RadarObstacles>();

      if (!radarProxy->channel_reader_) {
        QMessageBox::warning(this, tr("Create cyber Channel Reader"),
                             tr("There is no enough memory!!!\nCannot create "
                                "cyber channel reader!"),
                             QMessageBox::Ok);
        return;
      }

      auto radarcallback =
          [this, radarProxy](
              const std::shared_ptr<apollo::drivers::RadarObstacles>& pdata) {
            this->RadarRenderCallback(pdata, radarProxy);
          };

      std::string nodeName("Visualizer-");
      nodeName.append(radarProxy->root_item_.text(0).toStdString());

      if (!radarProxy->channel_reader_->InstallCallbackAndOpen(
              radarcallback,
              radarProxy->channel_name_combobox_.currentText().toStdString(),
              nodeName)) {
        QMessageBox::warning(
            this, tr("Setup Channel Callback"),
            tr("Channel Callback cannot be installed!!!\nPlease check it!"),
            QMessageBox::Ok);
        delete radarProxy->channel_reader_;
        radarProxy->channel_reader_ = nullptr;

        radarProxy->action_item_button_.setChecked(false);
        return;
      }
    }

    radarProxy->root_item_.setToolTip(
        0, radarProxy->channel_name_combobox_.currentText());

    radarProxy->action_item_button_.setText("Stop");
    radarProxy->channel_name_combobox_.setEnabled(false);

    radarProxy->reader_mutex_.unlock();
  } else {
    if (!radarProxy->channel_name_combobox_.isEnabled()) {
      radarProxy->reader_mutex_.lock();
      radarProxy->action_item_button_.setText("Play");
      radarProxy->channel_name_combobox_.setEnabled(true);
    }
  }
  ui_->treeWidget->setCurrentItem(nullptr);
  ui_->treeWidget->clearFocus();
}

void MainWindow::RadarRenderCallback(
    const std::shared_ptr<const apollo::drivers::RadarObstacles>& rawData,
    RadarData* radar) {
  radar->reader_mutex_.lock();
  radar->reader_mutex_.unlock();

  std::cout << "-------MainWindow::RadarRenderCallback()-------" << std::endl;

  if (rawData != nullptr) {
    RadarPoints* r = new RadarPoints(radar_points_shader_);
    if (r) {
      if (!r->FillData(rawData) ||
          !ui_->sceneWidget->AddTempRenderableObj(
              radar->root_item_.text(0).toStdString(), r)) {
        delete r;
      }
    } else {
      std::cerr << "cannot create RadarPoints Renderable Object" << std::endl;
    }
  }
}

void MainWindow::ActionOpenPointCloud(void) {
  if (pointcloud_shader_ == nullptr) {
    pointcloud_shader_ =
        RenderableObject::CreateShaderProgram(tr(pcVertexPath), tr(pcFragPath));
    if (pointcloud_shader_ != nullptr) {
      ui_->sceneWidget->AddNewShaderProg("pointcloud", pointcloud_shader_);
    } else {
      QMessageBox::warning(this, tr("NO Shader"),
                           tr("There is no suitable shader for Pointcloud!!!"),
                           QMessageBox::Ok);
      return;
    }
  }

  if (pointcloud_top_item_ == nullptr) {
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

  proxy->action_item_button_.setChecked(false);
  DoPlayVideoImage(false, proxy);

  proxy->video_image_viewer_.StartOrStopUpdate(false);
  proxy->root_item_.setHidden(true);
  proxy->video_image_viewer_.setVisible(false);
  ui_->videoImageGridLayout->removeWidget(&proxy->video_image_viewer_);

  video_image_viewer_list_.removeOne(proxy);
  closed_video_image_viewer_list_.append(proxy);

  if (video_image_viewer_list_.count()) {
    calculateWH();
  } else {
    ui_->videoImageWidget->setVisible(false);
  }
}

void MainWindow::UpdateActions(void) {
  QTreeWidgetItem* item = ui_->treeWidget->currentItem();
  ui_->actionDelImage->setEnabled(false);
  if (item) {
    if (!item->parent() && item != pointcloud_top_item_ &&
        item != all_channel_root_ && item != grid_root_item_) {
      ui_->actionDelImage->setEnabled(true);
    }
  }
}

void MainWindow::PointCloudReaderCallback(
    const std::shared_ptr<const apollo::drivers::PointCloud>& pdata) {
  pointcloud_reader_mutex_.lock();
  pointcloud_reader_mutex_.unlock();
  PointCloud* pc = new PointCloud(pdata->point_size(), 4, pointcloud_shader_);
  if (pc) {
    if (!pc->FillData(pdata) ||
        !ui_->sceneWidget->AddTempRenderableObj(pcTempObjGroupName, pc)) {
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
          this, tr("Setup Channel Name"),
          tr("Channel Name cannot be empty!!!\nPlease Select it!"),
          QMessageBox::Ok);
      pointcloud_button_->setChecked(false);
      return;
    }

    if (!pointcloud_channel_Reader_) {
      pointcloud_channel_Reader_ =
          new CyberChannReader<apollo::drivers::PointCloud>();

      if (!pointcloud_channel_Reader_) {
        QMessageBox::warning(this, tr("Create Cyber Channel Reader"),
                             tr("There is no enough memory!!!\nCannot create "
                                "cyber channel reader!"),
                             QMessageBox::Ok);
        pointcloud_button_->setChecked(false);
        return;
      }

      auto pointCallback =
          [this](const std::shared_ptr<apollo::drivers::PointCloud>& pdata) {
            this->PointCloudReaderCallback(pdata);
          };
      std::string nodeName("Visualizer-");
      nodeName.append(pointcloud_top_item_->text(0).toStdString());
      if (!pointcloud_channel_Reader_->InstallCallbackAndOpen(
              pointCallback, pointcloud_comboBox_->currentText().toStdString(),
              nodeName)) {
        QMessageBox::warning(
            this, tr("Setup Channel Callback"),
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
  } else {
    if (!pointcloud_comboBox_->isEnabled()) {
      pointcloud_reader_mutex_.lock();
      pointcloud_comboBox_->setEnabled(true);
      pointcloud_button_->setText(tr("Show"));
    }
  }

  ui_->treeWidget->setCurrentItem(nullptr);
  ui_->treeWidget->clearFocus();
}

void MainWindow::ImageReaderCallback(
    const std::shared_ptr<const apollo::drivers::Image>& imgData,
    VideoImgProxy* theVideoImgProxy) {
  theVideoImgProxy->reader_mutex_.lock();
  if (theVideoImgProxy->dynamic_texture_ != nullptr && imgData != nullptr) {
    if (theVideoImgProxy->dynamic_texture_->UpdateData(imgData)) {
      theVideoImgProxy->video_image_viewer_.SetupDynamicTexture(
          theVideoImgProxy->dynamic_texture_);
    } else {
      std::cerr << "--------Cannot update dynamic Texture Data--------"
                << std::endl;
    }
  } else {
    std::cerr
        << "----Dynamic Texture is nullptr or apollo.drivers.Image is nullptr"
        << std::endl;
  }
  theVideoImgProxy->reader_mutex_.unlock();
}

void MainWindow::ImageReaderCallback(
    const std::shared_ptr<const apollo::drivers::CompressedImage>& imgData,
    VideoImgProxy* theVideoImgProxy) {
  theVideoImgProxy->reader_mutex_.lock();
  if (theVideoImgProxy->dynamic_texture_ != nullptr && imgData != nullptr) {
    QImage img;
    const std::string& data = imgData->data();
    if (img.loadFromData(reinterpret_cast<const unsigned char*>(data.c_str()),
                         static_cast<int>(data.size()))) {
      if (theVideoImgProxy->dynamic_texture_->UpdateData(img)) {
        theVideoImgProxy->video_image_viewer_.SetupDynamicTexture(
            theVideoImgProxy->dynamic_texture_);
      } else {
        std::cerr << "--------Cannot update dynamic Texture Data--------"
                  << std::endl;
      }
    } else {
      std::cerr
          << "-----------Cannot load compressed image from data with QImage"
          << std::endl;
    }
  } else {
    std::cerr
        << "----Dynamic Texture is nullptr or apollo.drivers.Image is nullptr"
        << std::endl;
  }
  theVideoImgProxy->reader_mutex_.unlock();
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
          this, tr("Setup Channel Name"),
          tr("Channel Name cannot be empty!!!\nPlease select one!"),
          QMessageBox::Ok);
      theVideoImg->action_item_button_.setChecked(false);
      return;
    }

    const std::string channelName =
        theVideoImg->channel_name_combobox_.currentText().toStdString();
    if (_channelName2TypeMap[channelName] == CompressedImageType)
      theVideoImg->isCompressedImage_ = true;

    if (!theVideoImg->image_reader_) {
      if (theVideoImg->isCompressedImage_) {
        theVideoImg->compressed_image_reader_ =
            new CyberChannReader<apollo::drivers::CompressedImage>();
      } else {
        theVideoImg->image_reader_ =
            new CyberChannReader<apollo::drivers::Image>();
      }

      if (!theVideoImg->image_reader_) {
        QMessageBox::warning(this, tr("Create cyber Channel Reader"),
                             tr("There is no enough memory!!!\nCannot create "
                                "cyber channel reader!"),
                             QMessageBox::Ok);
        return;
      }

      std::string nodeName("Visualizer-");
      nodeName.append(theVideoImg->root_item_.text(0).toStdString());

      bool ret = false;

      if (theVideoImg->isCompressedImage_) {
        auto videoCallback =
            [this, theVideoImg](
                const std::shared_ptr<apollo::drivers::CompressedImage>&
                    pdata) { this->ImageReaderCallback(pdata, theVideoImg); };

        ret = theVideoImg->compressed_image_reader_->InstallCallbackAndOpen(
            videoCallback, channelName, nodeName);
      } else {
        auto videoCallback =
            [this, theVideoImg](
                const std::shared_ptr<apollo::drivers::Image>& pdata) {
              this->ImageReaderCallback(pdata, theVideoImg);
            };
        ret = theVideoImg->image_reader_->InstallCallbackAndOpen(
            videoCallback, channelName, nodeName);
      }
      if (!ret) {
        QMessageBox::warning(
            this, tr("Setup Channel Callback"),
            tr("Channel Callback cannot be installed!!!\nPlease check it!"),
            QMessageBox::Ok);

        theVideoImg->deleteReader();
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

    theVideoImg->reader_mutex_.unlock();
  } else {
    if (!theVideoImg->channel_name_combobox_.isEnabled()) {
      theVideoImg->reader_mutex_.lock();
      theVideoImg->action_item_button_.setText("Play");
      theVideoImg->channel_name_combobox_.setEnabled(true);
    }
  }
  ui_->treeWidget->setCurrentItem(nullptr);
  ui_->treeWidget->clearFocus();
}

void MainWindow::ChangePointCloudChannel() {
  if (pointcloud_channel_Reader_ != nullptr) {
    pointcloud_channel_Reader_->CloseChannel();
    std::string nodeName("Visualizer-");
    nodeName.append(pointcloud_top_item_->text(0).toStdString());
    pointcloud_channel_Reader_->OpenChannel(
        pointcloud_comboBox_->currentText().toStdString(), nodeName);
  }
}

void MainWindow::ChangeVideoImgChannel() {
  QComboBox* obj = static_cast<QComboBox*>(QObject::sender());
  VideoImgProxy* theVideoImg =
      StructPtrByMemberPtr(obj, VideoImgProxy, channel_name_combobox_);

  if (theVideoImg->image_reader_ != nullptr) {
    theVideoImg->deleteReader();
  }
}

void MainWindow::ChangeRadarChannel(void) {
  QComboBox* obj = static_cast<QComboBox*>(QObject::sender());
  RadarData* radar =
      StructPtrByMemberPtr(obj, RadarData, channel_name_combobox_);

  if (radar->channel_reader_ != nullptr) {
    radar->channel_reader_->CloseChannel();
    std::string nodeName("Visualizer-");
    nodeName.append(radar->root_item_.text(0).toStdString());
    radar->channel_reader_->OpenChannel(obj->currentText().toStdString(),
                                        nodeName);
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
    const apollo::cyber::proto::ChangeMsg& changeMsg) {
  if (apollo::cyber::proto::ChangeType::CHANGE_CHANNEL ==
          changeMsg.change_type() &&
      apollo::cyber::proto::RoleType::ROLE_WRITER == changeMsg.role_type() &&
      apollo::cyber::proto::OperateType::OPT_JOIN == changeMsg.operate_type()) {
    AddNewWriter(changeMsg.role_attr());
  }
}

void MainWindow::AddNewWriter(
    const apollo::cyber::proto::RoleAttributes& role) {
  const std::string& channelName = role.channel_name();
  if (_channelName2TypeMap.find(channelName) != _channelName2TypeMap.end()) {
    return;
  }
  const std::string& msgTypeName = role.message_type();
  _channelName2TypeMap[channelName] = msgTypeName;

  QTreeWidgetItem* child = new QTreeWidgetItem();
  if (child == nullptr) {
    QMessageBox::warning(this, tr("Error"), tr("No Enough for New Channel!!!"),
                         QMessageBox::Ok);
    return;
  }

  QString str(channelName.c_str());
  child->setText(0, str);
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

  QString msgType(msgTypeName.c_str());
  if (msgType.contains("camera", Qt::CaseInsensitive)) {
    for (VideoImgProxy* item : video_image_viewer_list_) {
      item->channel_name_combobox_.addItem(str);
    }

    for (VideoImgProxy* item : closed_video_image_viewer_list_) {
      item->channel_name_combobox_.addItem(str);
    }
  }

  if (msgType.contains("pointcloud", Qt::CaseInsensitive)) {
    pointcloud_comboBox_->addItem(str);
  }

  if (msgType.contains("radar", Qt::CaseInsensitive)) {
    for (RadarData* item : radarData_list_) {
      item->channel_name_combobox_.addItem(str);
    }

    for (RadarData* item : closed_radarData_list_) {
      item->channel_name_combobox_.addItem(str);
    }
  }
}

void MainWindow::PlayPause(void) {
  QObject* obj = QObject::sender();
  bool b = true;
  if (obj == ui_->actionPause) {
    b = false;
  }
  if (pointcloud_top_item_) {
    pointcloud_button_->setChecked(b);
    PlayRenderableObject(b);
  }
  for (VideoImgProxy* p : video_image_viewer_list_) {
    p->action_item_button_.setChecked(b);
    DoPlayVideoImage(b, p);
  }
  for (RadarData* item : radarData_list_) {
    item->action_item_button_.setChecked(b);
    DoOpenRadarChannel(b, item);
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
