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

#ifndef QBLUETOOTHDEVICEDISCOVERYAGENT_H
#define QBLUETOOTHDEVICEDISCOVERYAGENT_H

#include <QtBluetooth/qbluetoothglobal.h>

#include <QtCore/QObject>
#include <QtBluetooth/QBluetoothDeviceInfo>
#include <QtBluetooth/QBluetoothAddress>

QT_BEGIN_NAMESPACE

class QBluetoothDeviceDiscoveryAgentPrivate;

class Q_BLUETOOTH_EXPORT QBluetoothDeviceDiscoveryAgent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QBluetoothDeviceDiscoveryAgent::InquiryType inquiryType
               READ inquiryType WRITE setInquiryType)

public:
    // FIXME: add more errors
    // FIXME: add bluez error handling
    enum Error {
        NoError,
        InputOutputError,
        PoweredOffError,
        InvalidBluetoothAdapterError,
        UnsupportedPlatformError,
        UnknownError = 100 // New errors must be added before Unknown error
    };
    Q_ENUM(Error)

    enum InquiryType {
        GeneralUnlimitedInquiry,
        LimitedInquiry
    };
    Q_ENUM(InquiryType)

    QBluetoothDeviceDiscoveryAgent(QObject *parent = 0);
    explicit QBluetoothDeviceDiscoveryAgent(const QBluetoothAddress &deviceAdapter,
                                            QObject *parent = 0);
    ~QBluetoothDeviceDiscoveryAgent();

    // TODO Remove inquiry type in Qt 6 -> not really used anywhere
    QBluetoothDeviceDiscoveryAgent::InquiryType inquiryType() const;
    void setInquiryType(QBluetoothDeviceDiscoveryAgent::InquiryType type);

    bool isActive() const;

    Error error() const;
    QString errorString() const;

    QList<QBluetoothDeviceInfo> discoveredDevices() const;

public Q_SLOTS:
    void start();
    void stop();

Q_SIGNALS:
    void deviceDiscovered(const QBluetoothDeviceInfo &info);
    void finished();
    void error(QBluetoothDeviceDiscoveryAgent::Error error);
    void canceled();

private:
    Q_DECLARE_PRIVATE(QBluetoothDeviceDiscoveryAgent)
    QBluetoothDeviceDiscoveryAgentPrivate *d_ptr;

#ifdef QT_BLUEZ_BLUETOOTH
    Q_PRIVATE_SLOT(d_func(), void _q_deviceFound(const QString &address, const QVariantMap &dict))
    Q_PRIVATE_SLOT(d_func(), void _q_propertyChanged(const QString &name, const QDBusVariant &value))
    Q_PRIVATE_SLOT(d_func(), void _q_InterfacesAdded(const QDBusObjectPath &path, InterfaceList interfaceList))
    Q_PRIVATE_SLOT(d_func(), void _q_discoveryFinished())
    Q_PRIVATE_SLOT(d_func(), void _q_discoveryInterrupted(const QString &path))
    Q_PRIVATE_SLOT(d_func(), void _q_PropertiesChanged(const QString &interface, const QVariantMap &changed_properties, const QStringList &invalidated_properties))
    Q_PRIVATE_SLOT(d_func(), void _q_extendedDeviceDiscoveryTimeout())
#endif
};

QT_END_NAMESPACE

#endif
