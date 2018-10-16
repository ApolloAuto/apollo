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

#ifndef QBLUETOOTHSERVICEDISCOVERYAGENT_P_H
#define QBLUETOOTHSERVICEDISCOVERYAGENT_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include "qbluetoothaddress.h"
#include "qbluetoothdeviceinfo.h"
#include "qbluetoothserviceinfo.h"
#include "qbluetoothservicediscoveryagent.h"

#include <QStack>
#include <QStringList>

#ifdef QT_BLUEZ_BLUETOOTH
class OrgBluezManagerInterface;
class OrgBluezAdapterInterface;
class OrgBluezDeviceInterface;
class OrgFreedesktopDBusObjectManagerInterface;
#include <QtCore/qprocess.h>

QT_BEGIN_NAMESPACE
class QDBusPendingCallWatcher;
class QXmlStreamReader;
QT_END_NAMESPACE
#endif

#ifdef QT_QNX_BLUETOOTH
#include "qnx/ppshelpers_p.h"
#include <fcntl.h>
#include <unistd.h>
#include <QTimer>
#include <btapi/btdevice.h>
#endif

QT_BEGIN_NAMESPACE

class QBluetoothDeviceDiscoveryAgent;
#ifdef QT_ANDROID_BLUETOOTH
class ServiceDiscoveryBroadcastReceiver;
class LocalDeviceBroadcastReceiver;
#include <QtAndroidExtras/QAndroidJniObject>
#include <QtBluetooth/QBluetoothLocalDevice>
#endif

class QBluetoothServiceDiscoveryAgentPrivate
#ifdef QT_QNX_BLUETOOTH
: public QObject
{
    Q_OBJECT
#else
{
#endif
    Q_DECLARE_PUBLIC(QBluetoothServiceDiscoveryAgent)

public:
    enum DiscoveryState {
        Inactive,
        DeviceDiscovery,
        ServiceDiscovery,
    };

    QBluetoothServiceDiscoveryAgentPrivate(const QBluetoothAddress &deviceAdapter);
    ~QBluetoothServiceDiscoveryAgentPrivate();

    void startDeviceDiscovery();
    void stopDeviceDiscovery();
    void startServiceDiscovery();
    void stopServiceDiscovery();

    void setDiscoveryState(DiscoveryState s) { state = s; }
    inline DiscoveryState discoveryState() { return state; }

    void setDiscoveryMode(QBluetoothServiceDiscoveryAgent::DiscoveryMode m) { mode = m; }
    QBluetoothServiceDiscoveryAgent::DiscoveryMode DiscoveryMode() { return mode; }

    void _q_deviceDiscoveryFinished();
    void _q_deviceDiscovered(const QBluetoothDeviceInfo &info);
    void _q_serviceDiscoveryFinished();
    void _q_deviceDiscoveryError(QBluetoothDeviceDiscoveryAgent::Error);
#ifdef QT_BLUEZ_BLUETOOTH
    void _q_discoveredServices(QDBusPendingCallWatcher *watcher);
    void _q_createdDevice(QDBusPendingCallWatcher *watcher);
    //Slots below are used for discovering Bluetooth Low Energy devices. It will be used with Bluez 5.x version.
    /*
    void _g_discoveredGattService();
    void _q_discoverGattCharacteristics(QDBusPendingCallWatcher *watcher);
    void _q_discoveredGattCharacteristic(QDBusPendingCallWatcher *watcher);
    */
    void _q_sdpScannerDone(int exitCode, QProcess::ExitStatus status);
    void _q_finishSdpScan(QBluetoothServiceDiscoveryAgent::Error errorCode,
                          const QString &errorDescription,
                          const QStringList &xmlRecords);
#endif
#ifdef QT_ANDROID_BLUETOOTH
    void _q_processFetchedUuids(const QBluetoothAddress &address, const QList<QBluetoothUuid> &uuids);

    void populateDiscoveredServices(const QBluetoothDeviceInfo &remoteDevice,
                                    const QList<QBluetoothUuid> &uuids);
    void _q_fetchUuidsTimeout();
    void _q_hostModeStateChanged(QBluetoothLocalDevice::HostMode state);
#endif

private:
    void start(const QBluetoothAddress &address);
    void stop();
    bool isDuplicatedService(const QBluetoothServiceInfo &serviceInfo) const;

#ifdef QT_BLUEZ_BLUETOOTH
    void startBluez5(const QBluetoothAddress &address);
    void runExternalSdpScan(const QBluetoothAddress &remoteAddress,
                    const QBluetoothAddress localAddress);
    void sdpScannerDone(int exitCode, QProcess::ExitStatus exitStatus);
    QVariant readAttributeValue(QXmlStreamReader &xml);
    QBluetoothServiceInfo parseServiceXml(const QString& xml);
    void performMinimalServiceDiscovery(const QBluetoothAddress &deviceAddress);
#endif

#ifdef QT_QNX_BLUETOOTH
private slots:
    void remoteDevicesChanged(int fd);
    void controlReply(ppsResult result);
    void controlEvent(ppsResult result);
    void queryTimeout();
#ifdef QT_QNX_BT_BLUETOOTH
    static void deviceServicesDiscoveryCallback(bt_sdp_list_t *, void *, uint8_t);
#endif

private:
    int m_rdfd;
    QSocketNotifier *rdNotifier;
    QTimer m_queryTimer;
    bool m_btInitialized;
    bool m_serviceScanDone;
#endif

public:
    QBluetoothServiceDiscoveryAgent::Error error;
    QString errorString;
    QBluetoothAddress deviceAddress;
    QList<QBluetoothServiceInfo> discoveredServices;
    QList<QBluetoothDeviceInfo> discoveredDevices;
    QBluetoothAddress m_deviceAdapterAddress;

private:
    DiscoveryState state;
    QList<QBluetoothUuid> uuidFilter;

    QBluetoothDeviceDiscoveryAgent *deviceDiscoveryAgent;

    QBluetoothServiceDiscoveryAgent::DiscoveryMode mode;

    bool singleDevice;
#ifdef QT_BLUEZ_BLUETOOTH
    QString foundHostAdapterPath;
    OrgBluezManagerInterface *manager;
    OrgFreedesktopDBusObjectManagerInterface *managerBluez5;
    OrgBluezAdapterInterface *adapter;
    OrgBluezDeviceInterface *device;
    QProcess *sdpScannerProcess;
#endif

#ifdef QT_ANDROID_BLUETOOTH
    ServiceDiscoveryBroadcastReceiver *receiver;
    LocalDeviceBroadcastReceiver *localDeviceReceiver;

    QAndroidJniObject btAdapter;
    QMap<QBluetoothAddress,QPair<QBluetoothDeviceInfo,QList<QBluetoothUuid> > > sdpCache;
#endif

protected:
    QBluetoothServiceDiscoveryAgent *q_ptr;
};

QT_END_NAMESPACE

#endif
