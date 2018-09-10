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

#ifndef QBLUETOOTHSOCKET_H
#define QBLUETOOTHSOCKET_H

#include <QtBluetooth/qbluetoothglobal.h>

#include <QtBluetooth/QBluetoothAddress>
#include <QtBluetooth/QBluetoothUuid>
#include <QtBluetooth/QBluetoothServiceInfo>

#include <QtCore/QIODevice>
#include <QtNetwork/QAbstractSocket>

QT_BEGIN_NAMESPACE

class QBluetoothSocketPrivate;

class Q_BLUETOOTH_EXPORT QBluetoothSocket : public QIODevice
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QBluetoothSocket)

    friend class QBluetoothServer;
    friend class QBluetoothServerPrivate;

public:

    // TODO Decouple SocketState and SocketError enum values from QAbstractSocket in Qt 6
    enum SocketState {
        UnconnectedState = QAbstractSocket::UnconnectedState,
        ServiceLookupState = QAbstractSocket::HostLookupState,
        ConnectingState = QAbstractSocket::ConnectingState,
        ConnectedState = QAbstractSocket::ConnectedState,
        BoundState = QAbstractSocket::BoundState,
        ClosingState = QAbstractSocket::ClosingState,
        ListeningState = QAbstractSocket::ListeningState
    };
    Q_ENUM(SocketState)

    enum SocketError {
        NoSocketError = -2,
        UnknownSocketError = QAbstractSocket::UnknownSocketError, //-1
        HostNotFoundError = QAbstractSocket::HostNotFoundError, //2
        ServiceNotFoundError = QAbstractSocket::SocketAddressNotAvailableError, //9
        NetworkError = QAbstractSocket::NetworkError, //7
        UnsupportedProtocolError = 8,
        OperationError = QAbstractSocket::OperationError //19
        //New enums (independent of QAbstractSocket) should be added from 100 onwards
    };
    Q_ENUM(SocketError)

    explicit QBluetoothSocket(QBluetoothServiceInfo::Protocol socketType, QObject *parent = 0);   // create socket of type socketType
    QBluetoothSocket(QObject *parent = 0);  // create a blank socket
    virtual ~QBluetoothSocket();

    void abort();
    virtual void close();

    bool isSequential() const;

    virtual qint64 bytesAvailable() const;
    virtual qint64 bytesToWrite() const;

    virtual bool canReadLine() const;

    void connectToService(const QBluetoothServiceInfo &service, OpenMode openMode = ReadWrite);
    void connectToService(const QBluetoothAddress &address, const QBluetoothUuid &uuid, OpenMode openMode = ReadWrite);
    void connectToService(const QBluetoothAddress &address, quint16 port, OpenMode openMode = ReadWrite);
    void disconnectFromService();

    //bool flush();
    //bool isValid() const;

    QString localName() const;
    QBluetoothAddress localAddress() const;
    quint16 localPort() const;

    QString peerName() const;
    QBluetoothAddress peerAddress() const;
    quint16 peerPort() const;
    //QBluetoothServiceInfo peerService() const;

    //qint64 readBufferSize() const;
    //void setReadBufferSize(qint64 size);

    bool setSocketDescriptor(int socketDescriptor, QBluetoothServiceInfo::Protocol socketType,
                             SocketState socketState = ConnectedState,
                             OpenMode openMode = ReadWrite);
    int socketDescriptor() const;

    QBluetoothServiceInfo::Protocol socketType() const;
    SocketState state() const;
    SocketError error() const;
    QString errorString() const;

    //bool waitForConnected(int msecs = 30000);
    //bool waitForDisconnected(int msecs = 30000);
    //virtual bool waitForReadyRead(int msecs = 30000);

Q_SIGNALS:
    void connected();
    void disconnected();
    void error(QBluetoothSocket::SocketError error);
    void stateChanged(QBluetoothSocket::SocketState state);

protected:
    virtual qint64 readData(char *data, qint64 maxSize);
    virtual qint64 writeData(const char *data, qint64 maxSize);

    void setSocketState(SocketState state);
    void setSocketError(SocketError error);

    void doDeviceDiscovery(const QBluetoothServiceInfo &service, OpenMode openMode);

private Q_SLOTS:
    void serviceDiscovered(const QBluetoothServiceInfo &service);
    void discoveryFinished();


protected:
    QBluetoothSocketPrivate *d_ptr;

private:
    friend class QLowEnergyControllerPrivate;
};

#ifndef QT_NO_DEBUG_STREAM
Q_BLUETOOTH_EXPORT QDebug operator<<(QDebug, QBluetoothSocket::SocketError);
Q_BLUETOOTH_EXPORT QDebug operator<<(QDebug, QBluetoothSocket::SocketState);
#endif

QT_END_NAMESPACE

#endif
