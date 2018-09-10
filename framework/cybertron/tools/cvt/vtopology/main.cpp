#include <QApplication>
#include <QSurfaceFormat>
#include "cybertron/init.h"
#include "cybertron/topology/topology.h"
#include "main_window.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  MainWindow w;

  apollo::cybertron::Init(argv[0]);

  auto topologyCallback =
      [&w](const apollo::cybertron::proto::ChangeMsg& change_msg) {
        w.TopologyChanged(change_msg);
      };

  auto channelManager =
      apollo::cybertron::topology::Topology::Instance()->channel_manager();

  channelManager->AddChangeListener(topologyCallback);

  w.show();

  return a.exec();
}
