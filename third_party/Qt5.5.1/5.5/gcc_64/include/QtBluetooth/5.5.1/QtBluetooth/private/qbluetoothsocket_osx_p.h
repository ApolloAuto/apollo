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

#ifndef QBLUETOOTHSOCKET_OSX_P_H
#define QBLUETOOTHSOCKET_OSX_P_H

#ifdef QT_OSX_BLUETOOTH

#include "osx/osxbtchanneldelegate_p.h"
#include "osx/osxbtrfcommchannel_p.h"
#include "osx/osxbtl2capchannel_p.h"
#include "qbluetoothserviceinfo.h"
#include "osx/osxbtutility_p.h"
#include "qbluetoothsocket.h"

#ifndef QPRIVATELINEARBUFFER_BUFFERSIZE
#define QPRIVATELINEARBUFFER_BUFFERSIZE Q_INT64_C(16384)
#endif
// The order is important: bytearray before buffer:
#include <QtCore/qbytearray.h>
#include "qprivatelinearbuffer_p.h"

#include <QtCore/qscopedpointer.h>
#include <QtCore/qiodevice.h>
#include <QtCore/qglobal.h>
#include <QtCore/qobject.h>
#include <QtCore/qstring.h>
#include <QtCore/qvector.h>

@class IOBluetoothRFCOMMChannel;
@class IOBluetoothL2CAPChannel;

QT_BEGIN_NAMESPACE

class QBluetoothServiceDiscoveryAgent;
class QBluetoothAddress;

class QBluetoothSocketPrivate : public QBluetoothSocketPrivateBase, public OSXBluetooth::ChannelDelegate
{
    friend class QBluetoothSocket;
    friend class QBluetoothServer;

public:
    QBluetoothSocketPrivate();
    ~QBluetoothSocketPrivate();

    void connectToService(const QBluetoothAddress &address, quint16 port,
                          QIODevice::OpenMode openMode);

    void close();
    void abort();

    quint64 bytesAvailable() const;

    QString peerName() const;
    QBluetoothAddress peerAddress() const;
    quint16 peerPort() const;

    void _q_readNotify();
    void _q_writeNotify();

private:
    // Create a socket from an external source (without connectToService).
    bool setChannel(IOBluetoothRFCOMMChannel *channel);
    bool setChannel(IOBluetoothL2CAPChannel *channel);

    // L2CAP and RFCOMM delegate
    void setChannelError(IOReturn errorCode) Q_DECL_OVERRIDE;
    void channelOpenComplete() Q_DECL_OVERRIDE;
    void channelClosed() Q_DECL_OVERRIDE;
    void readChannelData(void *data, std::size_t size) Q_DECL_OVERRIDE;
    void writeComplete() Q_DECL_OVERRIDE;

    qint64 writeData(const char *data, qint64 maxSize);
    qint64 readData(char *data, qint64 maxSize);

    QBluetoothSocket *q_ptr;

    QScopedPointer<QBluetoothServiceDiscoveryAgent> discoveryAgent;

    QPrivateLinearBuffer buffer;
    QPrivateLinearBuffer txBuffer;
    QVector<char> writeChunk;

    // Probably, not needed.
    QIODevice::OpenMode openMode;

    QBluetoothSocket::SocketState state;
    QBluetoothServiceInfo::Protocol socketType;

    QBluetoothSocket::SocketError socketError;
    QString errorString;

    typedef QT_MANGLE_NAMESPACE(OSXBTL2CAPChannel) ObjCL2CAPChannel;
    typedef OSXBluetooth::ObjCScopedPointer<ObjCL2CAPChannel> L2CAPChannel;
    L2CAPChannel l2capChannel;

    typedef QT_MANGLE_NAMESPACE(OSXBTRFCOMMChannel) ObjCRFCOMMChannel;
    typedef OSXBluetooth::ObjCScopedPointer<ObjCRFCOMMChannel> RFCOMMChannel;
    RFCOMMChannel rfcommChannel;
    // A trick to deal with channel open too fast (synchronously).
    bool isConnecting;
};

QT_END_NAMESPACE

#endif

#endif
