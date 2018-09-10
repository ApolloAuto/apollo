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

#ifndef QBLUETOOTHSERVER_OSX_P_H
#define QBLUETOOTHSERVER_OSX_P_H

#ifdef QT_OSX_BLUETOOTH

#include "osx/osxbtsocketlistener_p.h"
#include "qbluetoothserviceinfo.h"
#include "osx/osxbtutility_p.h"
#include "qbluetoothserver.h"

#include <QtCore/qglobal.h>
#include <QtCore/qlist.h>

QT_BEGIN_NAMESPACE

class QMutex;

class QBluetoothServerPrivate : public OSXBluetooth::SocketListener
{
    friend class QBluetoothServer;
    friend class QBluetoothServiceInfoPrivate;

public:
    QBluetoothServerPrivate(QBluetoothServiceInfo::Protocol type, QBluetoothServer *q);
    ~QBluetoothServerPrivate();

    void _q_newConnection();
private:
    bool startListener(quint16 realPort);
    void stopListener();

    // SocketListener (delegate):
    void openNotify(IOBluetoothRFCOMMChannel *channel) Q_DECL_OVERRIDE;
    void openNotify(IOBluetoothL2CAPChannel *channel) Q_DECL_OVERRIDE;

    QBluetoothServiceInfo::Protocol serverType;
    QBluetoothServer *q_ptr;
    QBluetoothServer::Error lastError;

    // Either a "temporary" channelID/PSM assigned by QBluetoothServer::listen,
    // or a real channelID/PSM returned by IOBluetooth after we've registered
    // a service.
    quint16 port;

    typedef OSXBluetooth::ObjCScopedPointer<ObjCListener> Listener;
    Listener listener;

    int maxPendingConnections;

    // These static functions below
    // deal with differences between bluetooth sockets
    // (bluez and QtBluetooth's API) and IOBluetooth, where it's not possible
    // to have a real PSM/channelID _before_ a service is registered,
    // the solution - "fake" ports.
    // These functions require external locking - using channelMapMutex.
    static QMutex &channelMapMutex();

    static bool channelIsBusy(quint16 channelID);
    static quint16 findFreeChannel();

    static bool psmIsBusy(quint16 psm);
    static quint16 findFreePSM();

    static void registerServer(QBluetoothServerPrivate *server, quint16 port);
    static QBluetoothServerPrivate *registeredServer(quint16 port, QBluetoothServiceInfo::Protocol protocol);
    static void unregisterServer(QBluetoothServerPrivate *server);

    typedef OSXBluetooth::ObjCStrongReference<NSObject> PendingConnection;
    QList<PendingConnection> pendingConnections;

};

QT_END_NAMESPACE

#endif //QT_OSX_BLUETOOTH

#endif
