/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2013 BlackBerry Limited. All rights reserved.
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

#ifndef QBLUETOOTHSERVER_H
#define QBLUETOOTHSERVER_H

#include <QtBluetooth/qbluetoothglobal.h>

#include <QtCore/QObject>

#include <QtBluetooth/QBluetoothAddress>
#include <QtBluetooth/qbluetooth.h>
#include <QtBluetooth/QBluetoothSocket>
#include <QtBluetooth/QBluetoothServiceInfo>

QT_BEGIN_NAMESPACE

class QBluetoothServerPrivate;
class QBluetoothSocket;

class Q_BLUETOOTH_EXPORT QBluetoothServer : public QObject
{
    Q_OBJECT

public:
    enum Error {
        NoError,
        UnknownError,
        PoweredOffError,
        InputOutputError,
        ServiceAlreadyRegisteredError,
        UnsupportedProtocolError
    };
    Q_ENUM(Error)

    QBluetoothServer(QBluetoothServiceInfo::Protocol serverType, QObject *parent = 0);
    ~QBluetoothServer();

    void close();

    bool listen(const QBluetoothAddress &address = QBluetoothAddress(), quint16 port = 0);
    QBluetoothServiceInfo listen(const QBluetoothUuid &uuid, const QString &serviceName = QString());
    bool isListening() const;

    void setMaxPendingConnections(int numConnections);
    int maxPendingConnections() const;

    bool hasPendingConnections() const;
    QBluetoothSocket *nextPendingConnection();

    QBluetoothAddress serverAddress() const;
    quint16 serverPort() const;

    void setSecurityFlags(QBluetooth::SecurityFlags security);
    QBluetooth::SecurityFlags securityFlags() const;

    QBluetoothServiceInfo::Protocol serverType() const;

    Error error() const;

Q_SIGNALS:
    void newConnection();
    void error(QBluetoothServer::Error error);

protected:
    QBluetoothServerPrivate *d_ptr;

private:
    Q_DECLARE_PRIVATE(QBluetoothServer)

#ifdef QT_BLUEZ_BLUETOOTH
    Q_PRIVATE_SLOT(d_func(), void _q_newConnection())
#endif
};

QT_END_NAMESPACE

#endif
