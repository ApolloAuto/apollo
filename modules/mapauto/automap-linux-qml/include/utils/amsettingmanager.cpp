#include <QMutexLocker>
#include <QCoreApplication>
#include "amsettingmanager.h"

QSettings* AMSettingManager::_instance = 0;
QMutex AMSettingManager::_mutex;

QSettings* AMSettingManager::getInstance()
{
    if(!_instance)
    {
        QMutexLocker locker(&_mutex);
        if(!_instance)
        {
            QCoreApplication::setOrganizationName("Baidu");
            QCoreApplication::setOrganizationDomain("baidu.com");
            QCoreApplication::setApplicationName("AutoMap");

            // generate file:  ~/.config/Baidu/AutoMap.conf
            _instance = new QSettings;
        }
    }
    return _instance;
}

