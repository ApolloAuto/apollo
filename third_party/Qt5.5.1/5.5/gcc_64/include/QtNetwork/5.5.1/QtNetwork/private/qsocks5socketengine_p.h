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

#ifndef QSOCKS5SOCKETENGINE_P_H
#define QSOCKS5SOCKETENGINE_P_H

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

#include "qabstractsocketengine_p.h"
#include "qnetworkproxy.h"

QT_BEGIN_NAMESPACE

#ifndef QT_NO_SOCKS5

class QSocks5SocketEnginePrivate;

class Q_AUTOTEST_EXPORT QSocks5SocketEngine : public QAbstractSocketEngine
{
    Q_OBJECT
public:
    QSocks5SocketEngine(QObject *parent = 0);
    ~QSocks5SocketEngine();

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

private:
    Q_DECLARE_PRIVATE(QSocks5SocketEngine)
    Q_DISABLE_COPY(QSocks5SocketEngine)
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketConnected())
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketReadNotification())
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketError(QAbstractSocket::SocketError))
#ifndef QT_NO_UDPSOCKET
    Q_PRIVATE_SLOT(d_func(), void _q_udpSocketReadNotification())
#endif
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketBytesWritten())
    Q_PRIVATE_SLOT(d_func(), void _q_emitPendingReadNotification())
    Q_PRIVATE_SLOT(d_func(), void _q_emitPendingWriteNotification())
    Q_PRIVATE_SLOT(d_func(), void _q_emitPendingConnectionNotification())
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketDisconnected())
    Q_PRIVATE_SLOT(d_func(), void _q_controlSocketStateChanged(QAbstractSocket::SocketState))

};


class QTcpSocket;

class QSocks5Authenticator
{
public:
    QSocks5Authenticator();
    virtual ~QSocks5Authenticator();
    virtual char methodId();
    virtual bool beginAuthenticate(QTcpSocket *socket, bool *completed);
    virtual bool continueAuthenticate(QTcpSocket *socket, bool *completed);

    virtual bool seal(const QByteArray &buf, QByteArray *sealedBuf);
    virtual bool unSeal(const QByteArray &sealedBuf, QByteArray *buf);
    virtual bool unSeal(QTcpSocket *sealedSocket, QByteArray *buf);

    virtual QString errorString() { return QString(); }
};

class QSocks5PasswordAuthenticator : public QSocks5Authenticator
{
public:
    QSocks5PasswordAuthenticator(const QString &userName, const QString &password);
    char methodId() Q_DECL_OVERRIDE;
    bool beginAuthenticate(QTcpSocket *socket, bool *completed) Q_DECL_OVERRIDE;
    bool continueAuthenticate(QTcpSocket *socket, bool *completed) Q_DECL_OVERRIDE;

    QString errorString() Q_DECL_OVERRIDE;

private:
    QString userName;
    QString password;
};

struct QSocks5Data;
struct QSocks5ConnectData;
struct QSocks5UdpAssociateData;
struct QSocks5BindData;

class QSocks5SocketEnginePrivate : public QAbstractSocketEnginePrivate
{
    Q_DECLARE_PUBLIC(QSocks5SocketEngine)
public:
    QSocks5SocketEnginePrivate();
    ~QSocks5SocketEnginePrivate();

   enum Socks5State
    {
        Uninitialized = 0,
        ConnectError,
        AuthenticationMethodsSent,
        Authenticating,
        AuthenticatingError,
        RequestMethodSent,
        RequestError,
        Connected,
        UdpAssociateSuccess,
        BindSuccess,
        ControlSocketError,
        SocksError,
        HostNameLookupError
    };
    Socks5State socks5State;

    enum Socks5Mode
    {
        NoMode,
        ConnectMode,
        BindMode,
        UdpAssociateMode
    };
    Socks5Mode mode;

    enum Socks5Error
    {
        SocksFailure = 0x01,
        ConnectionNotAllowed = 0x02,
        NetworkUnreachable = 0x03,
        HostUnreachable = 0x04,
        ConnectionRefused = 0x05,
        TTLExpired = 0x06,
        CommandNotSupported = 0x07,
        AddressTypeNotSupported = 0x08,
        LastKnownError = AddressTypeNotSupported,
        UnknownError
    };

    void initialize(Socks5Mode socks5Mode);

    void setErrorState(Socks5State state, const QString &extraMessage = QString());
    void setErrorState(Socks5State state, Socks5Error socks5error);

    void reauthenticate();
    void parseAuthenticationMethodReply();
    void parseAuthenticatingReply();
    void sendRequestMethod();
    void parseRequestMethodReply();
    void parseNewConnection();

    bool waitForConnected(int msecs, bool *timedOut);

    void _q_controlSocketConnected();
    void _q_controlSocketReadNotification();
    void _q_controlSocketError(QAbstractSocket::SocketError);
#ifndef QT_NO_UDPSOCKET
    void checkForDatagrams() const;
    void _q_udpSocketReadNotification();
#endif
    void _q_controlSocketBytesWritten();
    void _q_controlSocketDisconnected();
    void _q_controlSocketStateChanged(QAbstractSocket::SocketState);

    QNetworkProxy proxyInfo;

    bool readNotificationEnabled, writeNotificationEnabled, exceptNotificationEnabled;

    qintptr socketDescriptor;

    QSocks5Data *data;
    QSocks5ConnectData *connectData;
#ifndef QT_NO_UDPSOCKET
    QSocks5UdpAssociateData *udpData;
#endif
    QSocks5BindData *bindData;
    QString peerName;
    QByteArray receivedHeaderFragment;

    mutable bool readNotificationActivated;
    mutable bool writeNotificationActivated;

    bool readNotificationPending;
    void _q_emitPendingReadNotification();
    void emitReadNotification();
    bool writeNotificationPending;
    void _q_emitPendingWriteNotification();
    void emitWriteNotification();
    bool connectionNotificationPending;
    void _q_emitPendingConnectionNotification();
    void emitConnectionNotification();
};

class Q_AUTOTEST_EXPORT QSocks5SocketEngineHandler : public QSocketEngineHandler
{
public:
    virtual QAbstractSocketEngine *createSocketEngine(QAbstractSocket::SocketType socketType,
                                                      const QNetworkProxy &, QObject *parent) Q_DECL_OVERRIDE;
    virtual QAbstractSocketEngine *createSocketEngine(qintptr socketDescriptor, QObject *parent) Q_DECL_OVERRIDE;
};


QT_END_NAMESPACE
#endif // QT_NO_SOCKS5
#endif // QSOCKS5SOCKETENGINE_H
