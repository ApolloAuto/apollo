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

#ifndef QWEBSOCKETSERVER_P_H
#define QWEBSOCKETSERVER_P_H
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

#include <QtCore/QObject>
#include <QtCore/QQueue>
#include <QtCore/QString>
#include <QtNetwork/QHostAddress>
#include <private/qobject_p.h>
#include "qwebsocket.h"

#ifndef QT_NO_SSL
#include <QtNetwork/QSslConfiguration>
#include <QtNetwork/QSslError>
#endif

QT_BEGIN_NAMESPACE

class QTcpServer;
class QWebSocketServer;

class QWebSocketServerPrivate : public QObjectPrivate
{
    Q_DISABLE_COPY(QWebSocketServerPrivate)

public:
    Q_DECLARE_PUBLIC(QWebSocketServer)
    enum SslMode
    {
        SecureMode = true,
        NonSecureMode
    };

    explicit QWebSocketServerPrivate(const QString &serverName, SslMode secureMode,
                                     QWebSocketServer * const pWebSocketServer);
    virtual ~QWebSocketServerPrivate();

    void init();
    void close(bool aboutToDestroy = false);
    QString errorString() const;
    bool hasPendingConnections() const;
    bool isListening() const;
    bool listen(const QHostAddress &address = QHostAddress::Any, quint16 port = 0);
    int maxPendingConnections() const;
    virtual QWebSocket *nextPendingConnection();
    void pauseAccepting();
#ifndef QT_NO_NETWORKPROXY
    QNetworkProxy proxy() const;
    void setProxy(const QNetworkProxy &networkProxy);
#endif
    void resumeAccepting();
    QHostAddress serverAddress() const;
    QWebSocketProtocol::CloseCode serverError() const;
    quint16 serverPort() const;
    void setMaxPendingConnections(int numConnections);
    bool setSocketDescriptor(qintptr socketDescriptor);
    qintptr socketDescriptor() const;

    QList<QWebSocketProtocol::Version> supportedVersions() const;
    QStringList supportedProtocols() const;
    QStringList supportedExtensions() const;

    void setServerName(const QString &serverName);
    QString serverName() const;

    SslMode secureMode() const;

#ifndef QT_NO_SSL
    void setSslConfiguration(const QSslConfiguration &sslConfiguration);
    QSslConfiguration sslConfiguration() const;
#endif

    void setError(QWebSocketProtocol::CloseCode code, const QString &errorString);

    QWebSocketServer * const q_ptr;

private:
    QTcpServer *m_pTcpServer;
    QString m_serverName;
    SslMode m_secureMode;
    QQueue<QWebSocket *> m_pendingConnections;
    QWebSocketProtocol::CloseCode m_error;
    QString m_errorString;
    int m_maxPendingConnections;

    void addPendingConnection(QWebSocket *pWebSocket);
    void setErrorFromSocketError(QAbstractSocket::SocketError error,
                                 const QString &errorDescription);

    void onNewConnection();
    void onCloseConnection();
    void handshakeReceived();
};

QT_END_NAMESPACE

#endif // QWEBSOCKETSERVER_P_H
