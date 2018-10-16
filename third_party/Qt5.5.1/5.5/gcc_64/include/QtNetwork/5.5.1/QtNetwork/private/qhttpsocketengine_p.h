/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNetwork module of the Qt Toolkit.
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

#ifndef QHTTPSOCKETENGINE_P_H
#define QHTTPSOCKETENGINE_P_H

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

#include "private/qabstractsocketengine_p.h"
#include "qabstractsocket.h"
#include "qnetworkproxy.h"
#include "private/qauthenticator_p.h"

QT_BEGIN_NAMESPACE

#if !defined(QT_NO_NETWORKPROXY) && !defined(QT_NO_HTTP)

class QTcpSocket;
class QHttpNetworkReply;
class QHttpSocketEnginePrivate;

class Q_AUTOTEST_EXPORT QHttpSocketEngine : public QAbstractSocketEngine
{
    Q_OBJECT
public:
    enum HttpState {
        None,
        ConnectSent,
        Connected,
        SendAuthentication,
        ReadResponseContent,
        ReadResponseHeader
    };
    QHttpSocketEngine(QObject *parent = 0);
    ~QHttpSocketEngine();

    bool initialize(QAbstractSocket::SocketType type, QAbstractSocket::NetworkLayerProtocol protocol = QAbstractSocket::IPv4Protocol) Q_DECL_OVERRIDE;
    bool initialize(qintptr socketDescriptor, QAbstractSocket::SocketState socketState = QAbstractSocket::ConnectedState) Q_DECL_OVERRIDE;

    void setProxy(const QNetworkProxy &networkProxy);

    qintptr socketDescriptor() const Q_DECL_OVERRIDE;

    bool isValid() const Q_DECL_OVERRIDE;

    bool connectInternal();
    bool connectToHost(const QHostAddress &address, quint16 port) Q_DECL_OVERRIDE;
    bool connectToHostByName(const QString &name, quint16 port) Q_DECL_OVERRIDE;
    bool bind(const QHostAddress &address, quint16 port) Q_DECL_OVERRIDE;
    bool listen() Q_DECL_OVERRIDE;
    int accept() Q_DECL_OVERRIDE;
    void close() Q_DECL_OVERRIDE;

    qint64 bytesAvailable() const Q_DECL_OVERRIDE;

    qint64 read(char *data, qint64 maxlen) Q_DECL_OVERRIDE;
    qint64 write(const char *data, qint64 len) Q_DECL_OVERRIDE;

#ifndef QT_NO_UDPSOCKET
#ifndef QT_NO_NETWORKINTERFACE
    bool joinMulticastGroup(const QHostAddress &groupAddress,
                            const QNetworkInterface &interface) Q_DECL_OVERRIDE;
    bool leaveMulticastGroup(const QHostAddress &groupAddress,
                             const QNetworkInterface &interface) Q_DECL_OVERRIDE;
    QNetworkInterface multicastInterface() const Q_DECL_OVERRIDE;
    bool setMulticastInterface(const QNetworkInterface &iface) Q_DECL_OVERRIDE;
#endif // QT_NO_NETWORKINTERFACE

    qint64 readDatagram(char *data, qint64 maxlen, QHostAddress *addr = 0,
        quint16 *port = 0) Q_DECL_OVERRIDE;
    qint64 writeDatagram(const char *data, qint64 len, const QHostAddress &addr,
        quint16 port) Q_DECL_OVERRIDE;
    bool hasPendingDatagrams() const Q_DECL_OVERRIDE;
    qint64 pendingDatagramSize() const Q_DECL_OVERRIDE;
#endif // QT_NO_UDPSOCKET

    qint64 bytesToWrite() const Q_DECL_OVERRIDE;

    int option(SocketOption option) const Q_DECL_OVERRIDE;
    bool setOption(SocketOption option, int value) Q_DECL_OVERRIDE;

    bool waitForRead(int msecs = 30000, bool *timedOut = 0) Q_DECL_OVERRIDE;
    bool waitForWrite(int msecs = 30000, bool *timedOut = 0) Q_DECL_OVERRIDE;
    bool waitForReadOrWrite(bool *readyToRead, bool *readyToWrite,
                            bool checkRead, bool checkWrite,
                            int msecs = 30000, bool *timedOut = 0) Q_DECL_OVERRIDE;

    bool isReadNotificationEnabled() const Q_DECL_OVERRIDE;
    void setReadNotificationEnabled(bool enable) Q_DECL_OVERRIDE;
    bool isWriteNotificationEnabled() const Q_DECL_OVERRIDE;
    void setWriteNotificationEnabled(bool enable) Q_DECL_OVERRIDE;
    bool isExceptionNotificationEnabled() const Q_DECL_OVERRIDE;
    void setExceptionNotificationEnabled(bool enable) Q_DECL_OVERRIDE;

public slots:
    void slotSocketConnected();
    void slotSocketDisconnected();
    void slotSocketReadNotification();
    void slotSocketBytesWritten();
    void slotSocketError(QAbstractSocket::SocketError error);
    void slotSocketStateChanged(QAbstractSocket::SocketState state);

private slots:
    void emitPendingReadNotification();
    void emitPendingWriteNotification();
    void emitPendingConnectionNotification();

private:
    void emitReadNotification();
    void emitWriteNotification();
    void emitConnectionNotification();

    bool readHttpHeader();

    Q_DECLARE_PRIVATE(QHttpSocketEngine)
    Q_DISABLE_COPY(QHttpSocketEngine)

};


class QHttpSocketEnginePrivate : public QAbstractSocketEnginePrivate
{
    Q_DECLARE_PUBLIC(QHttpSocketEngine)
public:
    QHttpSocketEnginePrivate();
    ~QHttpSocketEnginePrivate();

    QNetworkProxy proxy;
    QString peerName;
    QTcpSocket *socket;
    QHttpNetworkReply *reply; // only used for parsing the proxy response
    QHttpSocketEngine::HttpState state;
    QAuthenticator authenticator;
    bool readNotificationEnabled;
    bool writeNotificationEnabled;
    bool exceptNotificationEnabled;
    bool readNotificationActivated;
    bool writeNotificationActivated;
    bool readNotificationPending;
    bool writeNotificationPending;
    bool connectionNotificationPending;
    bool credentialsSent;
    uint pendingResponseData;
};

class Q_AUTOTEST_EXPORT QHttpSocketEngineHandler : public QSocketEngineHandler
{
public:
    virtual QAbstractSocketEngine *createSocketEngine(QAbstractSocket::SocketType socketType,
                                                      const QNetworkProxy &, QObject *parent) Q_DECL_OVERRIDE;
    virtual QAbstractSocketEngine *createSocketEngine(qintptr socketDescripter, QObject *parent) Q_DECL_OVERRIDE;
};
#endif

QT_END_NAMESPACE

#endif // QHTTPSOCKETENGINE_H
