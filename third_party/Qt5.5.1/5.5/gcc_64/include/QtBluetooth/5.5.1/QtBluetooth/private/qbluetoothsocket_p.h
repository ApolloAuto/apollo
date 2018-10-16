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

#ifndef QBLUETOOTHSOCKET_P_H
#define QBLUETOOTHSOCKET_P_H

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

#include "qbluetoothsocket.h"

#ifdef QT_QNX_BLUETOOTH
#include "qnx/ppshelpers_p.h"
#endif
#ifdef QT_ANDROID_BLUETOOTH
#include <QtAndroidExtras/QAndroidJniObject>
#include <QtCore/QPointer>
#include "android/inputstreamthread_p.h"
#include <jni.h>
class WorkerThread;
#endif

#ifndef QPRIVATELINEARBUFFER_BUFFERSIZE
#define QPRIVATELINEARBUFFER_BUFFERSIZE Q_INT64_C(16384)
#endif
#include "qprivatelinearbuffer_p.h"

#include <QtGlobal>

QT_FORWARD_DECLARE_CLASS(QSocketNotifier)

QT_BEGIN_NAMESPACE

class QBluetoothServiceDiscoveryAgent;

class QSocketServerPrivate
{
public:
    QSocketServerPrivate();
    ~QSocketServerPrivate();
};



class QBluetoothSocket;
class QBluetoothServiceDiscoveryAgent;

#ifndef QT_OSX_BLUETOOTH
class QBluetoothSocketPrivate : public QObject
{
    Q_OBJECT
    Q_DECLARE_PUBLIC(QBluetoothSocket)
    friend class QBluetoothServerPrivate;

public:

    QBluetoothSocketPrivate();
    ~QBluetoothSocketPrivate();

//On QNX and Android we connect using the uuid not the port
#if defined(QT_QNX_BLUETOOTH) || defined(QT_ANDROID_BLUETOOTH)
    void connectToService(const QBluetoothAddress &address, const QBluetoothUuid &uuid,
                          QIODevice::OpenMode openMode);
#else
    void connectToService(const QBluetoothAddress &address, quint16 port, QIODevice::OpenMode openMode);
#endif
#ifdef QT_ANDROID_BLUETOOTH
    bool fallBackConnect(QAndroidJniObject uuid, int channel);
#endif


    bool ensureNativeSocket(QBluetoothServiceInfo::Protocol type);

    QString localName() const;
    QBluetoothAddress localAddress() const;
    quint16 localPort() const;

    QString peerName() const;
    QBluetoothAddress peerAddress() const;
    quint16 peerPort() const;
    //QBluetoothServiceInfo peerService() const;

    void abort();
    void close();

    //qint64 readBufferSize() const;
    //void setReadBufferSize(qint64 size);

    qint64 writeData(const char *data, qint64 maxSize);
    qint64 readData(char *data, qint64 maxSize);

#ifdef QT_ANDROID_BLUETOOTH
    bool setSocketDescriptor(const QAndroidJniObject &socket, QBluetoothServiceInfo::Protocol socketType,
                             QBluetoothSocket::SocketState socketState = QBluetoothSocket::ConnectedState,
                             QBluetoothSocket::OpenMode openMode = QBluetoothSocket::ReadWrite);
#endif
    bool setSocketDescriptor(int socketDescriptor, QBluetoothServiceInfo::Protocol socketType,
                             QBluetoothSocket::SocketState socketState = QBluetoothSocket::ConnectedState,
                             QBluetoothSocket::OpenMode openMode = QBluetoothSocket::ReadWrite);

    qint64 bytesAvailable() const;

public:
    QPrivateLinearBuffer buffer;
    QPrivateLinearBuffer txBuffer;
    int socket;
    QBluetoothServiceInfo::Protocol socketType;
    QBluetoothSocket::SocketState state;
    QBluetoothSocket::SocketError socketError;
    QSocketNotifier *readNotifier;
    QSocketNotifier *connectWriteNotifier;
    bool connecting;

    QBluetoothServiceDiscoveryAgent *discoveryAgent;
    QBluetoothSocket::OpenMode openMode;


//    QByteArray rxBuffer;
//    qint64 rxOffset;
    QString errorString;

#ifdef QT_ANDROID_BLUETOOTH
    QAndroidJniObject adapter;
    QAndroidJniObject socketObject;
    QAndroidJniObject remoteDevice;
    QAndroidJniObject inputStream;
    QAndroidJniObject outputStream;
    InputStreamThread *inputThread;

public slots:
    void socketConnectSuccess(const QAndroidJniObject &socket);
    void defaultSocketConnectFailed(const QAndroidJniObject & socket,
                                    const QAndroidJniObject &targetUuid);
    void fallbackSocketConnectFailed(const QAndroidJniObject &socket,
                                     const QAndroidJniObject &targetUuid);
    void inputThreadError(int errorCode);

signals:
    void connectJavaSocket();
    void closeJavaSocket();

#endif

#if defined(QT_QNX_BLUETOOTH) || defined(QT_BLUEZ_BLUETOOTH)
private slots:
    void _q_readNotify();
    void _q_writeNotify();
#endif

protected:
    QBluetoothSocket *q_ptr;

private:
#ifdef QT_QNX_BLUETOOTH
    QBluetoothAddress m_peerAddress;
    QBluetoothUuid m_uuid;
    bool isServerSocket;

private slots:
    void controlReply(ppsResult result);
    void controlEvent(ppsResult result);
#endif

#ifdef QT_BLUEZ_BLUETOOTH
public:
    quint8 lowEnergySocketType;
#endif
};

#else // QT_OSX_BLUETOOTH

// QBluetoothSocketPrivate on OS X can not contain
// Q_OBJECT (moc does not parse Objective-C syntax).
// But QBluetoothSocket still requires QMetaObject::invokeMethod
// to work. Here's the trick:
class QBluetoothSocketPrivateBase : public QObject
{
// The most important part of it:
    Q_OBJECT
public slots:
    virtual void _q_writeNotify() = 0;
};

#endif // QT_OSX_BLUETOOTH

static inline void convertAddress(quint64 from, quint8 (&to)[6])
{
    to[0] = (from >> 0) & 0xff;
    to[1] = (from >> 8) & 0xff;
    to[2] = (from >> 16) & 0xff;
    to[3] = (from >> 24) & 0xff;
    to[4] = (from >> 32) & 0xff;
    to[5] = (from >> 40) & 0xff;
}

static inline void convertAddress(quint8 (&from)[6], quint64 &to)
{
    to = (quint64(from[0]) << 0) |
         (quint64(from[1]) << 8) |
         (quint64(from[2]) << 16) |
         (quint64(from[3]) << 24) |
         (quint64(from[4]) << 32) |
         (quint64(from[5]) << 40);
}

QT_END_NAMESPACE


#endif
