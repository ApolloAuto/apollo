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

#pragma once

#include <QtCore/QMutex>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/radar.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/tools/visualizer/channel_reader.h"
#include "modules/tools/visualizer/msg_dialog.h"

class FixedAspectRatioWidget;
class Texture;
class Grid;
class QAction;
class QComboBox;
class QTreeWidgetItem;
class QOpenGLShaderProgram;
class QCheckBox;
class QColorDialog;
class VideoImagesDialog;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  void TopologyChanged(const apollo::cyber::proto::ChangeMsg& change_msg);
  void AddNewWriter(const apollo::cyber::proto::RoleAttributes& role);

 protected:
  void resizeEvent(QResizeEvent*) override;

 private slots:
  void ActionAddGrid(void);

  void ActionOpenPointCloud(void);
  void PlayRenderableObject(bool);
  void ChangePointCloudChannel(void);

  void ActionOpenImage(void);
  void PlayVideoImage(bool b);
  void ChangeVideoImgChannel(void);
  void SelectCurrentTreeItem(FixedAspectRatioWidget*);

  void ActionDelVideoImage(void);

  void CloseVideoImgViewer(bool b);

  void UpdateActions(void);
  void EnableGrid(bool b);

  void EditGridColor(QTreeWidgetItem* item, int column);
  void ChangeGridCellCountBySize(int v);

  void ActionOpenImages(void);
  void AddVideoImages(void);

  void ActionOpenRadarChannel(void);
  void openRadarChannel(bool b);
  void EnableRadarPoints(bool b);
  void ChangeRadarChannel(void);

  void showMessage(void);

  void PlayPause(void);

 private:
  struct VideoImgProxy;
  struct RadarData;

  void PointCloudReaderCallback(
      const std::shared_ptr<const apollo::drivers::PointCloud>& pdata);
  void ImageReaderCallback(
      const std::shared_ptr<const apollo::drivers::Image>& imgData,
      VideoImgProxy* proxy);
  void ImageReaderCallback(
      const std::shared_ptr<const apollo::drivers::CompressedImage>& imgData,
      VideoImgProxy* proxy);

  void InsertAllChannelNames(void);
  VideoImgProxy* AddVideoImgViewer(void);
  void DoDeleteVideoImg(VideoImgProxy*);
  void DoPlayVideoImage(bool, VideoImgProxy*);
  void calculateWH(void);

  RadarData* createRadarData(void);
  void DoOpenRadarChannel(bool b, RadarData* radarProxy);
  void RadarRenderCallback(
      const std::shared_ptr<const apollo::drivers::RadarObstacles>& rawData,
      RadarData* radar);

  Ui::MainWindow* ui_;
  MessageDialog* msg_dialog_;
  VideoImagesDialog* open_images_dialog_;

  QTreeWidgetItem* all_channel_root_;

  Grid* grid_;
  QCheckBox* enable_grid_checkBox_;
  QTreeWidgetItem* grid_root_item_;

  QTreeWidgetItem* pointcloud_top_item_;
  QComboBox* pointcloud_comboBox_;
  QPushButton* pointcloud_button_;
  CyberChannReader<apollo::drivers::PointCloud>* pointcloud_channel_Reader_;

  QMutex pointcloud_reader_mutex_;

  QMenu right_menu_;

  std::shared_ptr<QOpenGLShaderProgram> pointcloud_shader_;
  std::shared_ptr<QOpenGLShaderProgram> grid_shader_;
  std::shared_ptr<QOpenGLShaderProgram> radar_points_shader_;

  QList<VideoImgProxy*> video_image_viewer_list_;
  QList<VideoImgProxy*> closed_video_image_viewer_list_;

  QList<RadarData*> radarData_list_;
  QList<RadarData*> closed_radarData_list_;

  std::map<std::string, std::string> _channelName2TypeMap;
};
