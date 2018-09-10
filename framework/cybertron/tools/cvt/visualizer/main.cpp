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

#include <QApplication>
#include <QFile>
#include <QSurfaceFormat>
#include "cybertron/init.h"
#include "main_window.h"

int main(int argc, char* argv[]) {
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setRenderableType(QSurfaceFormat::RenderableType::OpenGL);
  format.setSwapBehavior(QSurfaceFormat::SwapBehavior::DoubleBuffer);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication a(argc, argv);

  a.setStyleSheet(
      "QSplitter::handle {"
      "   background: lightGray;"
      "   border-radius: 2px; "
      "}");

  apollo::cybertron::Init(argv[0]);

  MainWindow w;

  auto topologyCallback =
      [&w](const apollo::cybertron::proto::ChangeMsg& change_msg) {
        w.TopologyChanged(change_msg);
      };

  apollo::cybertron::topology::Topology::Instance()
      ->channel_manager()
      ->AddChangeListener(topologyCallback);
  w.show();

  return a.exec();
}
