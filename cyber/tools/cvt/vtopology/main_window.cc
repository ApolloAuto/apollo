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

#include "./main_window.h"
#include "./arrow.h"
#include "cyber/init.h"
#include "cyber/proto/topology_change.pb.h"
#include "ui_main_window.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QGraphicsScene>
#include <QMessageBox>
#include <QTreeWidgetItem>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#define MEMBER_OFFSET(StructType, Member)                                    \
  (size_t)(                                                                  \
      reinterpret_cast<char*>(&(reinterpret_cast<StructType*>(1)->Member)) - \
      1)
#define StructPtrByMemberPtr(MemberPtr, StructType, Member)          \
  reinterpret_cast<StructType*>(reinterpret_cast<char*>(MemberPtr) - \
                                MEMBER_OFFSET(StructType, Member))

namespace {
inline void updateMaxVec(const QRectF& rect, QVector2D& v) {
  if (rect.width() > v.x()) {
    v.setX(rect.width());
  }

  if (rect.height() > v.y()) {
    v.setY(rect.height());
  }
}
}  // namespace

struct MainWindow::ChannelData {
  CompositeItem _sceneItem;
  QTreeWidgetItem _treeItem;
  ChannelData(const ::QString& text, ::CompositeItem::ItemType itemType);
};

MainWindow::ChannelData::ChannelData(const ::QString& text,
                                     ::CompositeItem::ItemType itemType)
    : _sceneItem(itemType, text), _treeItem() {
  _treeItem.setText(0, text);
}

struct MainWindow::ChannelDataSet {
  MainWindow::ChannelData* _channelData;
  std::vector<MainWindow::ChannelData*> _reader_nodes;
  std::vector<MainWindow::ChannelData*> _writer_nodes;
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow),
      about_dialog_(new AboutDialog),
      focused_item_(nullptr),
      topology_scene_(new QGraphicsScene(this)),
      refresh_timer_(this),
      item_raw_color_(Qt::black),
      max_rect_(-1, -1) {
  ui_->setupUi(this);

  {
    QStringList tmp;
    tmp << "Channels";
    all_channel_root_ = new QTreeWidgetItem(tmp);
    ui_->treeWidget->addTopLevelItem(all_channel_root_);

    tmp.clear();
    tmp << "Readers";
    all_reader_root_ = new QTreeWidgetItem(tmp);
    ui_->treeWidget->addTopLevelItem(all_reader_root_);

    tmp.clear();
    tmp << "Writers";
    all_writer_root_ = new QTreeWidgetItem(tmp);
    ui_->treeWidget->addTopLevelItem(all_writer_root_);
  }

  topology_scene_->setSceneRect(QRectF(0, 0, 2000, 2000));
  ui_->graphicsView->setScene(topology_scene_);

  connect(ui_->actionAbout, SIGNAL(triggered()), about_dialog_, SLOT(show()));
  connect(ui_->treeWidget, SIGNAL(doubleClicked(QModelIndex)), this,
          SLOT(TreeItemChanged(void)));
  connect(topology_scene_, SIGNAL(selectionChanged()), this,
          SLOT(SceneItemSelectionChanged(void)));

  refresh_timer_.setInterval(40);
  connect(&refresh_timer_, SIGNAL(timeout()), this, SLOT(UpdateSceneItem()));
  refresh_timer_.start();
}

MainWindow::~MainWindow() {
  apollo::cyber::Shutdown();

  all_channel_root_->takeChildren();
  all_reader_root_->takeChildren();
  all_writer_root_->takeChildren();

  topology_scene_->clear();

  for (auto iter = channel_reader_writer_map_.begin();
       iter != channel_reader_writer_map_.end(); ++iter) {
    delete iter->second;
  }

  delete ui_;
  delete about_dialog_;
}

void MainWindow::TopologyChanged(
    const apollo::cyber::proto::ChangeMsg& change_msg) {
  const std::string& channelName = change_msg.role_attr().channel_name();
  const std::string& nodeName = change_msg.role_attr().node_name();

  ChangeSceneItemFunc changeSceneFunc = &MainWindow::AddSceneItem;

  if (change_msg.operate_type() ==
      apollo::cyber::proto::OperateType::OPT_LEAVE) {
    changeSceneFunc = &MainWindow::RemoveSceneItem;
  }

  if (change_msg.change_type() ==
      apollo::cyber::proto::ChangeType::CHANGE_CHANNEL) {
    if (change_msg.role_type() ==
        apollo::cyber::proto::RoleType::ROLE_WRITER) {
      (this->*changeSceneFunc)(channelName, nodeName, false);
    }

    if (change_msg.role_type() ==
        apollo::cyber::proto::RoleType::ROLE_READER) {
      (this->*changeSceneFunc)(channelName, nodeName, true);
    }
  }
}

inline Arrow* MainWindow::createArrow(CompositeItem* startItem,
                                      CompositeItem* endItem) {
  Arrow* a = new Arrow(startItem, endItem);
  if (a) {
    startItem->AddArrow(a);
    endItem->AddArrow(a);

    buffer_list_.append(a);
  }
  return a;
}

inline void MainWindow::delSceneItemAndArrow(MainWindow::ChannelData* item) {
  if (item) {
    delete_list_.append(item);
  }
}

void MainWindow::UpdateSceneItem() {
  if (!buffer_list_.isEmpty() || !delete_list_.isEmpty()) {
    AdjustSceneLayout();
  }

  while (!buffer_list_.isEmpty()) {
    topology_scene_->addItem(buffer_list_.takeFirst());
  }
  while (!delete_list_.isEmpty()) {
    ChannelData* item = delete_list_.takeFirst();
    item->_sceneItem.setVisible(false);
    item->_sceneItem.removeArrows();
    topology_scene_->removeItem(&(item->_sceneItem));
    delete item;
  }

  topology_scene_->update();
}

void MainWindow::AddSceneItem(const std::string& channelName,
                              const std::string& nodeName, bool isReader) {
  auto iter = channel_reader_writer_map_.find(channelName);
  if (iter == channel_reader_writer_map_.cend()) {
    auto dataSet = new ChannelDataSet();
    if (dataSet == nullptr) return;

    dataSet->_channelData =
        new ChannelData(QString(channelName.c_str()), CompositeItem::Channel);
    if (dataSet->_channelData == nullptr) {
      delete dataSet;
      return;
    }

    ChannelData* nodeData =
        new ChannelData(QString(nodeName.c_str()), CompositeItem::Node);
    if (nodeData == nullptr) {
      delete dataSet;
      return;
    }

    updateMaxVec(dataSet->_channelData->_sceneItem.boundingRect(), max_rect_);

    if (isReader) {
      dataSet->_reader_nodes.push_back(nodeData);
      createArrow(&(dataSet->_channelData->_sceneItem),
                  &(nodeData->_sceneItem));
      all_reader_root_->addChild(&(nodeData->_treeItem));
    } else {
      dataSet->_writer_nodes.push_back(nodeData);
      createArrow(&(nodeData->_sceneItem),
                  &(dataSet->_channelData->_sceneItem));
      all_writer_root_->addChild(&(nodeData->_treeItem));
    }

    updateMaxVec(nodeData->_sceneItem.boundingRect(), max_rect_);

    all_channel_root_->addChild(&(dataSet->_channelData->_treeItem));

    buffer_list_.append(&(dataSet->_channelData->_sceneItem));
    buffer_list_.append(&(nodeData->_sceneItem));

    channel_reader_writer_map_[channelName] = dataSet;
  } else {
    ChannelData* nodeData =
        new ChannelData(QString(nodeName.c_str()), CompositeItem::Node);

    if (nodeData == nullptr) {
      return;
    }

    if (isReader) {
      iter->second->_reader_nodes.push_back(nodeData);
      createArrow(&(iter->second->_channelData->_sceneItem),
                  &(nodeData->_sceneItem));
      all_reader_root_->addChild(&(nodeData->_treeItem));
    } else {
      iter->second->_writer_nodes.push_back(nodeData);
      createArrow(&(nodeData->_sceneItem),
                  &(iter->second->_channelData->_sceneItem));
      all_writer_root_->addChild(&(nodeData->_treeItem));
    }
    updateMaxVec(nodeData->_sceneItem.boundingRect(), max_rect_);
    buffer_list_.append(&(nodeData->_sceneItem));
  }

  ui_->treeWidget->setRootIsDecorated(true);
}

void MainWindow::RemoveSceneItem(const std::string& channelName,
                                 const std::string& nodeName, bool isReader) {
  auto iter = channel_reader_writer_map_.find(channelName);
  if (iter != channel_reader_writer_map_.cend()) {
    ChannelData* item = nullptr;

    std::vector<ChannelData*>* nodesPtr = isReader
                                              ? &(iter->second->_reader_nodes)
                                              : &(iter->second->_writer_nodes);

    for (auto nodeIter = nodesPtr->begin(); nodeIter != nodesPtr->end();
         ++nodeIter) {
      if ((*nodeIter)->_sceneItem.text().toStdString() == nodeName) {
        item = *nodeIter;
        *nodeIter = nullptr;
        nodesPtr->erase(nodeIter);
        break;
      }
    }

    if (item) {
      QTreeWidgetItem* root = isReader ? all_reader_root_ : all_writer_root_;
      root->removeChild(&(item->_treeItem));

      delSceneItemAndArrow(item);
    }

    if (!iter->second->_reader_nodes.size() &&
        !iter->second->_writer_nodes.size()) {
      all_channel_root_->removeChild(&(iter->second->_channelData->_treeItem));

      delSceneItemAndArrow(iter->second->_channelData);

      iter->second = nullptr;
      channel_reader_writer_map_.erase(iter);
    }
  }
}

void MainWindow::AdjustSceneLayout(void) {
  int index = 0;
  int baseH = 0;

  int maxCellHeight = max_rect_.y();
  int maxCellWidth = max_rect_.x();
  maxCellWidth += 5;
  int w2 = (topology_scene_->width() - maxCellWidth) / 2;

  for (auto iter = channel_reader_writer_map_.begin();
       iter != channel_reader_writer_map_.end(); ++iter, ++index) {
    unsigned s = iter->second->_reader_nodes.size();
    if (s < iter->second->_writer_nodes.size())
      s = iter->second->_writer_nodes.size();
    s /= 2;
    ++s;

    baseH = baseH + s * maxCellHeight;
    iter->second->_channelData->_sceneItem.setPos(w2, baseH);

    int rwH, rwIndex;
    int w = w2 - maxCellWidth / 3 -
            (iter->second->_writer_nodes.size() + 1) / 2 * maxCellWidth;
    rwIndex = 0;
    auto rwIter = iter->second->_writer_nodes.begin();
    for (; rwIter != iter->second->_writer_nodes.end(); ++rwIter, ++rwIndex) {
      rwH = (rwIndex + 1) / 2 * maxCellHeight;
      if (rwIndex & 1)
        rwH = baseH - rwH;
      else
        rwH = baseH + rwH;

      (*rwIter)->_sceneItem.setPos(w + (rwIndex + 1) / 2 * maxCellWidth, rwH);
    }

    w = w2 + maxCellWidth / 3 +
        (iter->second->_reader_nodes.size() + 1) / 2 * maxCellWidth;
    rwIndex = 0;
    rwIter = iter->second->_reader_nodes.begin();
    for (; rwIter != iter->second->_reader_nodes.end(); ++rwIter, ++rwIndex) {
      rwH = (rwIndex + 1) / 2 * maxCellHeight;
      if (rwIndex & 1)
        rwH = baseH - rwH;
      else
        rwH = baseH + rwH;

      (*rwIter)->_sceneItem.setPos(w - (rwIndex + 1) / 2 * maxCellWidth, rwH);
    }

    baseH += s * maxCellHeight;
  }
}

void MainWindow::TreeItemChanged(void) {
  QTreeWidgetItem* item = ui_->treeWidget->currentItem();
  if (item) {
    if (item == all_channel_root_) return;
    if (item == all_reader_root_) return;
    if (item == all_writer_root_) return;

    ChannelData* channelData =
        StructPtrByMemberPtr(item, ChannelData, _treeItem);

    if (focused_item_ == &channelData->_sceneItem) {
      return;
    }

    if (focused_item_) {
      focused_item_->SetPenColor(item_raw_color_);
    }

    focused_item_ = &channelData->_sceneItem;

    item_raw_color_ = focused_item_->CurrentPenColor();
    focused_item_->SetPenColor(Qt::green);
    topology_scene_->setFocusItem(focused_item_);
    ui_->graphicsView->centerOn(focused_item_);
  }
}

void MainWindow::SceneItemSelectionChanged(void) {
  QGraphicsItem* item = topology_scene_->mouseGrabberItem();
  if (item == nullptr) {
    ui_->graphicsView->clearFocus();
    ui_->treeWidget->clearFocus();
    return;
  }

  item = item->parentItem();
  if (item == nullptr) return;
  if (item == focused_item_) return;

  if (focused_item_) {
    focused_item_->SetPenColor(item_raw_color_);
  }

  focused_item_ = static_cast<CompositeItem*>(item);
  item_raw_color_ = focused_item_->OldPenColor();
  focused_item_->SetPenColor(Qt::green);

  ChannelData* channelData =
      StructPtrByMemberPtr(focused_item_, ChannelData, _sceneItem);
  channelData->_treeItem.parent()->setExpanded(true);
  ui_->treeWidget->setCurrentItem(&channelData->_treeItem);
}
