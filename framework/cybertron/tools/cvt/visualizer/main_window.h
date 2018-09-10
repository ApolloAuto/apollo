/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMutex>
#include <QPixmap>

#include <sensor_image.pb.h>
#include <sensor_pointcloud.pb.h>
#include <memory>

#include "channel_reader.h"
#include "msg_dialog.h"

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

  void TopologyChanged(const apollo::cybertron::proto::ChangeMsg& change_msg);

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

  void showMessage(void);

  void PlayPause(void);

 private:
  struct VideoImgProxy;

  void PointCloudReaderCallback(
      const std::shared_ptr<const adu::common::sensor::PointCloud>& pdata);
  void ImageReaderCallback(
      const std::shared_ptr<const adu::common::sensor::CompressedImage>&
          imgData,
      VideoImgProxy* proxy);

  void InsertAllChannelNames(void);
  VideoImgProxy* AddVideoImgViewer(void);
  void DoDeleteVideoImg(VideoImgProxy*);
  void DoPlayVideoImage(bool, VideoImgProxy*);
  void calculateWH(/*VideoImgProxy* videoImgProxy*/);

  Ui::MainWindow* ui_;
  MessageDialog* msg_dialog_;
  VideoImagesDialog* open_images_dialog_;

  QTreeWidgetItem* all_channel_root_;

  Grid* grid_;
  QCheckBox* enable_grid_checkBox_;
  QTreeWidgetItem* grid_root_Item_;

  QTreeWidgetItem* pointcloud_top_item_;
  QComboBox* pointcloud_comboBox_;
  QPushButton* pointcloud_button_;
  CyberChannReader<adu::common::sensor::PointCloud>* pointcloud_channel_Reader_;

  QMutex pointcloud_reader_mutex_;

  QMenu right_menu_;

  std::shared_ptr<QOpenGLShaderProgram> pointcloud_shader_;
  std::shared_ptr<QOpenGLShaderProgram> grid_shader_;

  QList<VideoImgProxy*> video_image_viewer_list_;
  QList<VideoImgProxy*> closed_video_image_viewer_list_;
};

#endif  // MAINWINDOW_H
