#include <QApplication>
#include <QQmlApplicationEngine>
#include <QTextCodec>
#include <QQuickView>
//#include "Cube.h"
#include "src/BaiduMapAutoQmlInterface.h"
#include "utils/def.h"

QT_USE_NAMESPACE
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForLocale(codec);
    QCoreApplication::setOrganizationName("Baidu");
    QCoreApplication::setOrganizationDomain("baidu.com");
    QCoreApplication::setApplicationName("BDMapAuto");
//    BNAInit();
    QQmlApplicationEngine engine;
    qmlRegisterType<AUTO_MAP::BaiduMapAutoQmlInterface>( "BaiduMapAuto", 1, 0, "BaiduMapAutoQmlInterface" );

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    return app.exec();

}
