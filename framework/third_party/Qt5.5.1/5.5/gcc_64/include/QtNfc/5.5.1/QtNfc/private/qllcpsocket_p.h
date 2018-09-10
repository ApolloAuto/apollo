/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNfc module of the Qt Toolkit.
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

#ifndef QLLCPSOCKET_H
#define QLLCPSOCKET_H

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

#include <QtCore/QIODevice>
#include <QtNetwork/QAbstractSocket>
#include <QtNfc/qnfcglobal.h>

QT_BEGIN_NAMESPACE

class QNearFieldTarget;
class QLlcpSocketPrivate;

class Q_NFC_EXPORT QLlcpSocket : public QIODevice
{
    Q_OBJECT

    Q_DECLARE_PRIVATE(QLlcpSocket)

    friend class QLlcpServerPrivate;

public:
    enum SocketState {
        UnconnectedState = QAbstractSocket::UnconnectedState,
        ConnectingState = QAbstractSocket::ConnectingState,
        ConnectedState = QAbstractSocket::ConnectedState,
        ClosingState = QAbstractSocket::ClosingState,
        BoundState = QAbstractSocket::BoundState,
        ListeningState = QAbstractSocket::ListeningState
    };
    Q_ENUM(SocketState)

    enum SocketError {
        UnknownSocketError = QAbstractSocket::UnknownSocketError,
        RemoteHostClosedError = QAbstractSocket::RemoteHostClosedError,
        SocketAccessError = QAbstractSocket::SocketAccessError,
        SocketResourceError = QAbstractSocket::SocketResourceError
    };
    Q_ENUM(SocketError)

    explicit QLlcpSocket(QObject *parent = 0);
    ~QLlcpSocket();

    void connectToService(QNearFieldTarget *target, const QString &serviceUri);
    void disconnectFromService();

    void close();

    bool bind(quint8 port);

    bool hasPendingDatagrams() const;
    qint64 pendingDatagramSize() const;

    qint64 writeDatagram(const char *data, qint64 size);
    qint64 writeDatagram(const QByteArray &datagram);

    qint64 readDatagram(char *data, qint64 maxSize,
                        QNearFieldTarget **target = 0, quint8 *port = 0);
    qint64 writeDatagram(const char *data, qint64 size,
                         QNearFieldTarget *target, quint8 port);
    qint64 writeDatagram(const QByteArray &datagram, QNearFieldTarget *target, quint8 port);

    SocketError error() const;
    SocketState state() const;

    qint64 bytesAvailable() const;
    bool canReadLine() const;

    bool waitForReadyRead(int msecs = 30000);
    bool waitForBytesWritten(int msecs = 30000);
    virtual bool waitForConnected(int msecs = 30000);
    virtual bool waitForDisconnected(int msecs = 30000);
    bool isSequential() const;

signals:
    void connected();
    void disconnected();
    void error(QLlcpSocket::SocketError socketError);
    void stateChanged(QLlcpSocket::SocketState socketState);

protected:
    qint64 readData(char *data, qint64 maxlen);
    qint64 writeData(const char *data, qint64 len);

private:
    QLlcpSocket(QLlcpSocketPrivate *d, QObject *parent);

    QLlcpSocketPrivate *d_ptr;
};

QT_END_NAMESPACE

#endif // QLLCPSOCKET_H
