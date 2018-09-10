/****************************************************************************
**
** Copyright (C) 2014 Kurt Pattyn <pattyn.kurt@gmail.com>.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWebSockets module of the Qt Toolkit.
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

#ifndef QWEBSOCKETSERVER_H
#define QWEBSOCKETSERVER_H

#include "QtWebSockets/qwebsockets_global.h"
#include "QtWebSockets/qwebsocketprotocol.h"

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtNetwork/QHostAddress>

#ifndef QT_NO_SSL
#include <QtNetwork/QSslConfiguration>
#include <QtNetwork/QSslError>
#endif

QT_BEGIN_NAMESPACE

class QWebSocketServerPrivate;
class QWebSocket;
class QWebSocketCorsAuthenticator;

class Q_WEBSOCKETS_EXPORT QWebSocketServer : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(QWebSocketServer)
    Q_DECLARE_PRIVATE(QWebSocketServer)

    Q_ENUMS(SslMode)

public:
    enum SslMode {
#ifndef QT_NO_SSL
        SecureMode,
#endif
        NonSecureMode
    };

    explicit QWebSocketServer(const QString &serverName, SslMode secureMode,
                              QObject *parent = 0);
    virtual ~QWebSocketServer();

    bool listen(const QHostAddress &address = QHostAddress::Any, quint16 port = 0);
    void close();

    bool isListening() const;

    void setMaxPendingConnections(int numConnections);
    int maxPendingConnections() const;

    quint16 serverPort() const;
    QHostAddress serverAddress() const;
    QUrl serverUrl() const;

    SslMode secureMode() const;

    bool setSocketDescriptor(int socketDescriptor);
    int socketDescriptor() const;

    bool hasPendingConnections() const;
    virtual QWebSocket *nextPendingConnection();

    QWebSocketProtocol::CloseCode error() const;
    QString errorString() const;

    void pauseAccepting();
    void resumeAccepting();

    void setServerName(const QString &serverName);
    QString serverName() const;

#ifndef QT_NO_NETWORKPROXY
    void setProxy(const QNetworkProxy &networkProxy);
    QNetworkProxy proxy() const;
#endif
#ifndef QT_NO_SSL
    void setSslConfiguration(const QSslConfiguration &sslConfiguration);
    QSslConfiguration sslConfiguration() const;
#endif

    QList<QWebSocketProtocol::Version> supportedVersions() const;

Q_SIGNALS:
    void acceptError(QAbstractSocket::SocketError socketError);
    void serverError(QWebSocketProtocol::CloseCode closeCode);
    //TODO: should use a delegate iso of a synchronous signal
    //see also QTBUG-16251
    void originAuthenticationRequired(QWebSocketCorsAuthenticator *pAuthenticator);
    void newConnection();
#ifndef QT_NO_SSL
    void peerVerifyError(const QSslError &error);
    void sslErrors(const QList<QSslError> &errors);
#endif
    void closed();
};

QT_END_NAMESPACE

#endif // QWEBSOCKETSERVER_H
