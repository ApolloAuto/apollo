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

#ifndef TOOLS_CVT_VTOPOLOGY_MAINWINDOW_H_
#define TOOLS_CVT_VTOPOLOGY_MAINWINDOW_H_

#include <QColor>
#include <QList>
#include <QMainWindow>
#include <QMutex>
#include <QPixmap>
#include <QTimer>
#include <QVector2D>
#include <map>
#include <memory>
#include <string>

#include "./about_dialog.h"
#include "./composite_item.h"

class QTreeWidgetItem;

class GraphicsScene;
class QGraphicsItem;
class Arrow;

namespace Ui {
class MainWindow;
}

namespace apollo {
namespace cyber {
namespace proto {
class ChangeMsg;
}
}  // namespace cyber
}  // namespace apollo

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  void TopologyChanged(const apollo::cyber::proto::ChangeMsg& change_msg);

 private slots:
  void UpdateSceneItem(void);
  void TreeItemChanged(void);
  void SceneItemSelectionChanged(void);

 private:
  struct ChannelData;

  void AddSceneItem(const std::string& channelName, const std::string& nodeName,
                    bool isReader);
  void RemoveSceneItem(const std::string& channelName,
                       const std::string& nodeName, bool isReader);
  using ChangeSceneItemFunc =
      void (MainWindow::*)(const std::string& channelName,
                           const std::string& nodeName, bool isReader);

  Arrow* createArrow(CompositeItem* startItem, CompositeItem* endItem);
  void delSceneItemAndArrow(ChannelData* item);

  void AdjustSceneLayout(void);

  Ui::MainWindow* ui_;
  AboutDialog* about_dialog_;

  QTreeWidgetItem* all_channel_root_;
  QTreeWidgetItem* all_reader_root_;
  QTreeWidgetItem* all_writer_root_;
  CompositeItem* focused_item_;

  QGraphicsScene* topology_scene_;
  QTimer refresh_timer_;

  QColor item_raw_color_;
  QVector2D max_rect_;

  QList<QGraphicsItem*> buffer_list_;
  QList<ChannelData*> delete_list_;

  struct ChannelDataSet;
  std::map<std::string, ChannelDataSet*> channel_reader_writer_map_;
};

#endif  // MAINWINDOW_H
