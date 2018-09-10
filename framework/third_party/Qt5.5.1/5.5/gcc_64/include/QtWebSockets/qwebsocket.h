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

#ifndef QWEBSOCKET_H
#define QWEBSOCKET_H

#include <QtCore/QUrl>
#include <QtNetwork/QAbstractSocket>
#ifndef QT_NO_NETWORKPROXY
#include <QtNetwork/QNetworkProxy>
#endif
#ifndef QT_NO_SSL
#include <QtNetwork/QSslError>
#include <QtNetwork/QSslConfiguration>
#endif
#include "QtWebSockets/qwebsockets_global.h"
#include "QtWebSockets/qwebsocketprotocol.h"

QT_BEGIN_NAMESPACE

class QTcpSocket;
class QWebSocketPrivate;
class QMaskGenerator;

class Q_WEBSOCKETS_EXPORT QWebSocket : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(QWebSocket)
    Q_DECLARE_PRIVATE(QWebSocket)

public:
    explicit QWebSocket(const QString &origin = QString(),
                        QWebSocketProtocol::Version version = QWebSocketProtocol::VersionLatest,
                        QObject *parent = 0);
    virtual ~QWebSocket();

    void abort();
    QAbstractSocket::SocketError error() const;
    QString errorString() const;
    bool flush();
    bool isValid() const;
    QHostAddress localAddress() const;
    quint16 localPort() const;
    QAbstractSocket::PauseModes pauseMode() const;
    QHostAddress peerAddress() const;
    QString peerName() const;
    quint16 peerPort() const;
#ifndef QT_NO_NETWORKPROXY
    QNetworkProxy proxy() const;
    void setProxy(const QNetworkProxy &networkProxy);
#endif
    void setMaskGenerator(const QMaskGenerator *maskGenerator);
    const QMaskGenerator *maskGenerator() const;
    qint64 readBufferSize() const;
    void setReadBufferSize(qint64 size);

    void resume();
    void setPauseMode(QAbstractSocket::PauseModes pauseMode);

    QAbstractSocket::SocketState state() const;

    QWebSocketProtocol::Version version() const;
    QString resourceName() const;
    QUrl requestUrl() const;
    QString origin() const;
    QWebSocketProtocol::CloseCode closeCode() const;
    QString closeReason() const;

    qint64 sendTextMessage(const QString &message);
    qint64 sendBinaryMessage(const QByteArray &data);

#ifndef QT_NO_SSL
    void ignoreSslErrors(const QList<QSslError> &errors);
    void setSslConfiguration(const QSslConfiguration &sslConfiguration);
    QSslConfiguration sslConfiguration() const;
#endif

public Q_SLOTS:
    void close(QWebSocketProtocol::CloseCode closeCode = QWebSocketProtocol::CloseCodeNormal,
               const QString &reason = QString());
    void open(const QUrl &url);
    void ping(const QByteArray &payload = QByteArray());
#ifndef QT_NO_SSL
    void ignoreSslErrors();
#endif

Q_SIGNALS:
    void aboutToClose();
    void connected();
    void disconnected();
    void stateChanged(QAbstractSocket::SocketState state);
#ifndef QT_NO_NETWORKPROXY
    void proxyAuthenticationRequired(const QNetworkProxy &proxy, QAuthenticator *pAuthenticator);
#endif
    void readChannelFinished();
    void textFrameReceived(const QString &frame, bool isLastFrame);
    void binaryFrameReceived(const QByteArray &frame, bool isLastFrame);
    void textMessageReceived(const QString &message);
    void binaryMessageReceived(const QByteArray &message);
    void error(QAbstractSocket::SocketError error);
    void pong(quint64 elapsedTime, const QByteArray &payload);
    void bytesWritten(qint64 bytes);

#ifndef QT_NO_SSL
    void sslErrors(const QList<QSslError> &errors);
#endif

private:
    QWebSocket(QTcpSocket *pTcpSocket, QWebSocketProtocol::Version version,
               QObject *parent = Q_NULLPTR);
};

QT_END_NAMESPACE

#endif // QWEBSOCKET_H
