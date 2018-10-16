/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2013 Javier S. Pedro <maemo@javispedro.com>
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
#ifndef QLOWENERGYSERVICE_H
#define QLOWENERGYSERVICE_H

#include <QtBluetooth/QBluetoothAddress>
#include <QtBluetooth/QBluetoothUuid>
#include <QtBluetooth/QLowEnergyCharacteristic>

QT_BEGIN_NAMESPACE

class QLowEnergyServicePrivate;
class QLowEnergyControllerPrivate;
class Q_BLUETOOTH_EXPORT QLowEnergyService : public QObject
{
    Q_OBJECT
public:
    enum ServiceType {
        PrimaryService = 0x0001,
        IncludedService = 0x0002
    };
    Q_ENUM(ServiceType)
    Q_DECLARE_FLAGS(ServiceTypes, ServiceType)

    enum ServiceError {
        NoError = 0,
        OperationError,
        CharacteristicWriteError,
        DescriptorWriteError,
        UnknownError,
        CharacteristicReadError,
        DescriptorReadError
    };
    Q_ENUM(ServiceError)

    enum ServiceState {
        InvalidService = 0,
        DiscoveryRequired,  // we know start/end handle but nothing more
        //TODO Rename DiscoveringServices -> DiscoveringDetails or DiscoveringService
        DiscoveringServices,// discoverDetails() called and running
        ServiceDiscovered   // all details have been synchronized
    };
    Q_ENUM(ServiceState)

    enum WriteMode {
        WriteWithResponse = 0,
        WriteWithoutResponse
    };
    Q_ENUM(WriteMode)

    ~QLowEnergyService();

    QList<QBluetoothUuid> includedServices() const;

    QLowEnergyService::ServiceTypes type() const;
    QLowEnergyService::ServiceState state() const;

    QLowEnergyCharacteristic characteristic(const QBluetoothUuid &uuid) const;
    QList<QLowEnergyCharacteristic> characteristics() const;
    QBluetoothUuid serviceUuid() const;
    QString serviceName() const;

    void discoverDetails();

    ServiceError error() const;

    bool contains(const QLowEnergyCharacteristic &characteristic) const;
    void readCharacteristic(const QLowEnergyCharacteristic &characteristic);
    void writeCharacteristic(const QLowEnergyCharacteristic &characteristic,
                             const QByteArray &newValue,
                             WriteMode mode = WriteWithResponse);

    bool contains(const QLowEnergyDescriptor &descriptor) const;
    void readDescriptor(const QLowEnergyDescriptor &descriptor);
    void writeDescriptor(const QLowEnergyDescriptor &descriptor,
                         const QByteArray &newValue);

Q_SIGNALS:
    void stateChanged(QLowEnergyService::ServiceState newState);
    void characteristicChanged(const QLowEnergyCharacteristic &info,
                               const QByteArray &value);
    void characteristicRead(const QLowEnergyCharacteristic &info,
                            const QByteArray &value);
    void characteristicWritten(const QLowEnergyCharacteristic &info,
                               const QByteArray &value);
    void descriptorRead(const QLowEnergyDescriptor &info,
                        const QByteArray &value);
    void descriptorWritten(const QLowEnergyDescriptor &info,
                           const QByteArray &value);
    void error(QLowEnergyService::ServiceError error);

private:
    Q_DECLARE_PRIVATE(QLowEnergyService)
    QSharedPointer<QLowEnergyServicePrivate> d_ptr;

    // QLowEnergyController is the factory for this class
    friend class QLowEnergyController;
    QLowEnergyService(QSharedPointer<QLowEnergyServicePrivate> p,
                      QObject *parent = 0);
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QLowEnergyService::ServiceTypes)

QT_END_NAMESPACE

#endif // QLOWENERGYSERVICE_H
