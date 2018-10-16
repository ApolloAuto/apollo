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

#ifndef QLOWENERGYCONTROLLER_H
#define QLOWENERGYCONTROLLER_H

#include <QtCore/QObject>
#include <QtBluetooth/QBluetoothAddress>
#include <QtBluetooth/QBluetoothDeviceInfo>
#include <QtBluetooth/QBluetoothUuid>
#include <QtBluetooth/QLowEnergyService>

QT_BEGIN_NAMESPACE

class QLowEnergyControllerPrivate;
class Q_BLUETOOTH_EXPORT QLowEnergyController : public QObject
{
    Q_OBJECT
public:
    enum Error {
        NoError,
        UnknownError,
        UnknownRemoteDeviceError,
        NetworkError,
        InvalidBluetoothAdapterError,
        ConnectionError
    };
    Q_ENUM(Error)

    enum ControllerState {
        UnconnectedState = 0,
        ConnectingState,
        ConnectedState,
        DiscoveringState,
        DiscoveredState,
        ClosingState
    };
    Q_ENUM(ControllerState)

    enum RemoteAddressType {
        PublicAddress = 0,
        RandomAddress
    };
    Q_ENUM(RemoteAddressType)

    explicit QLowEnergyController(const QBluetoothAddress &remoteDevice,
                                     QObject *parent = 0); // TODO Qt 6 remove ctor
    explicit QLowEnergyController(const QBluetoothDeviceInfo &remoteDevice,
                                     QObject *parent = 0);
    explicit QLowEnergyController(const QBluetoothAddress &remoteDevice,
                                     const QBluetoothAddress &localDevice,
                                     QObject *parent = 0); // TODO Qt 6 remove ctor
    ~QLowEnergyController();

    QBluetoothAddress localAddress() const;
    QBluetoothAddress remoteAddress() const;

    QString remoteName() const;

    ControllerState state() const;

    RemoteAddressType remoteAddressType() const;
    void setRemoteAddressType(RemoteAddressType type);

    void connectToDevice();
    void disconnectFromDevice();

    void discoverServices();
    QList<QBluetoothUuid> services() const;
    QLowEnergyService *createServiceObject(
            const QBluetoothUuid &service, QObject *parent = 0);

    Error error() const;
    QString errorString() const;

Q_SIGNALS:
    void connected();
    void disconnected();
    void stateChanged(QLowEnergyController::ControllerState state);
    void error(QLowEnergyController::Error newError);

    void serviceDiscovered(const QBluetoothUuid &newService);
    void discoveryFinished();

private:
    Q_DECLARE_PRIVATE(QLowEnergyController)
    QLowEnergyControllerPrivate *d_ptr;
};

QT_END_NAMESPACE

#endif // QLOWENERGYCONTROLLER_H
