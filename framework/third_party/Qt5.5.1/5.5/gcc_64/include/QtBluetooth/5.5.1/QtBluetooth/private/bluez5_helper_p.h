/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef BLUEZ5_HELPER_H
#define BLUEZ5_HELPER_H

#include <QtCore/QObject>
#include <QtDBus/QtDBus>
#include <QtBluetooth/QBluetoothAddress>

typedef QMap<QString, QVariantMap> InterfaceList;
typedef QMap<QDBusObjectPath, InterfaceList> ManagedObjectList;

Q_DECLARE_METATYPE(InterfaceList)
Q_DECLARE_METATYPE(ManagedObjectList)

QT_BEGIN_NAMESPACE

bool isBluez5();

QString findAdapterForAddress(const QBluetoothAddress &wantedAddress, bool *ok);

class QtBluezDiscoveryManagerPrivate;
class QtBluezDiscoveryManager : public QObject
{
    Q_OBJECT
public:
    QtBluezDiscoveryManager(QObject* parent = 0);
    ~QtBluezDiscoveryManager();
    static QtBluezDiscoveryManager *instance();

    bool registerDiscoveryInterest(const QString &adapterPath);
    void unregisterDiscoveryInterest(const QString &adapterPath);

    //void dumpState() const;

signals:
    void discoveryInterrupted(const QString &adapterPath);

private slots:
    void InterfacesRemoved(const QDBusObjectPath &object_path,
                           const QStringList &interfaces);
    void PropertiesChanged(const QString &interface,
                           const QVariantMap &changed_properties,
                           const QStringList &invalidated_properties);

private:
    void removeAdapterFromMonitoring(const QString &dbusPath);

    QtBluezDiscoveryManagerPrivate *d;
};

QT_END_NAMESPACE

#endif
