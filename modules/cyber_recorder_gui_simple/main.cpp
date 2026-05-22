#include "mainwindow.h"

#include "cyber/init.h"

#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ::apollo::cyber::Init("cyber_recorder_gui_simple", "cyber_recorder_gui_simple");
    MainWindow w;
    w.show();
    const int result = a.exec();
    ::apollo::cyber::Clear();
    return result;
}
