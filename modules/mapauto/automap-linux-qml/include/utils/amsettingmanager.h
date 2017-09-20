#ifndef AMSETTINGMANAGER_H
#define AMSETTINGMANAGER_H

#include <QMutex>
#include <QSettings>

class AMSettingManager
{
public:
    static QSettings* getInstance();

private:
    static QSettings *_instance;
    static QMutex _mutex;

private:
    AMSettingManager();
    AMSettingManager(const AMSettingManager &); //禁止拷贝构造函数
    AMSettingManager &operator = (const AMSettingManager &); //禁止赋值拷贝函数
};

#endif // AMSETTINGMANAGER_H
