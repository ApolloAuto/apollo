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

#ifndef QBLUETOOTHSERVICEINFO_H
#define QBLUETOOTHSERVICEINFO_H

#include <QtBluetooth/qbluetoothglobal.h>

#include <QtBluetooth/QBluetoothUuid>
#include <QtBluetooth/QBluetoothAddress>

#include <QtCore/QMetaType>
#include <QtCore/QList>
#include <QtCore/QSharedPointer>
#include <QtCore/QVariant>

#include <QtCore/QDebug>

QT_BEGIN_NAMESPACE

class QBluetoothServiceInfoPrivate;
class QBluetoothDeviceInfo;

class Q_BLUETOOTH_EXPORT QBluetoothServiceInfo
{
public:
    enum AttributeId {
        ServiceRecordHandle = 0x0000,
        ServiceClassIds = 0x0001,
        ServiceRecordState = 0x0002,
        ServiceId = 0x0003,
        ProtocolDescriptorList = 0x0004,
        BrowseGroupList = 0x0005,
        LanguageBaseAttributeIdList = 0x0006,
        ServiceInfoTimeToLive = 0x0007,
        ServiceAvailability = 0x0008,
        BluetoothProfileDescriptorList = 0x0009,
        DocumentationUrl = 0x000A,
        ClientExecutableUrl = 0x000B,
        IconUrl = 0x000C,
        AdditionalProtocolDescriptorList = 0x000D,
        PrimaryLanguageBase = 0x0100,
        ServiceName = PrimaryLanguageBase + 0x0000,
        ServiceDescription = PrimaryLanguageBase + 0x0001,
        ServiceProvider = PrimaryLanguageBase + 0x0002
    };

    enum Protocol {
        UnknownProtocol,
        L2capProtocol,
        RfcommProtocol
    };

    class Sequence : public QList<QVariant>
    {
    public:
        Sequence() { }
        Sequence(const QList<QVariant> &list) : QList<QVariant>(list) { }
    };

    class Alternative : public QList<QVariant>
    {
    public:
        Alternative() { }
        Alternative(const QList<QVariant> &list) : QList<QVariant>(list) { }
    };

    QBluetoothServiceInfo();
    QBluetoothServiceInfo(const QBluetoothServiceInfo &other);
    ~QBluetoothServiceInfo();

    bool isValid() const;
    bool isComplete() const;

    void setDevice(const QBluetoothDeviceInfo &info);
    QBluetoothDeviceInfo device() const;

    void setAttribute(quint16 attributeId, const QVariant &value);
    void setAttribute(quint16 attributeId, const QBluetoothUuid &value);
    void setAttribute(quint16 attributeId, const QBluetoothServiceInfo::Sequence &value);
    void setAttribute(quint16 attributeId, const QBluetoothServiceInfo::Alternative &value);
    QVariant attribute(quint16 attributeId) const;
    QList<quint16> attributes() const;
    bool contains(quint16 attributeId) const;
    void removeAttribute(quint16 attributeId);

    inline void setServiceName(const QString &name);
    inline QString serviceName() const;
    inline void setServiceDescription(const QString &description);
    inline QString serviceDescription() const;
    inline void setServiceProvider(const QString &provider);
    inline QString serviceProvider() const;

    QBluetoothServiceInfo::Protocol socketProtocol() const;
    int protocolServiceMultiplexer() const;
    int serverChannel() const;

    QBluetoothServiceInfo::Sequence protocolDescriptor(QBluetoothUuid::ProtocolUuid protocol) const;

    inline void setServiceAvailability(quint8 availability);
    inline quint8 serviceAvailability() const;

    inline void setServiceUuid(const QBluetoothUuid &uuid);
    inline QBluetoothUuid serviceUuid() const;

    QList<QBluetoothUuid> serviceClassUuids() const;

    QBluetoothServiceInfo &operator=(const QBluetoothServiceInfo &other);

    bool isRegistered() const;
    bool registerService(const QBluetoothAddress &localAdapter = QBluetoothAddress());
    bool unregisterService();

protected:
    friend Q_BLUETOOTH_EXPORT QDebug operator<<(QDebug, const QBluetoothServiceInfo &);

protected:
    QSharedPointer<QBluetoothServiceInfoPrivate> d_ptr;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QBluetoothServiceInfo)
Q_DECLARE_METATYPE(QBluetoothServiceInfo::Sequence)
Q_DECLARE_METATYPE(QBluetoothServiceInfo::Alternative)

QT_BEGIN_NAMESPACE

inline void QBluetoothServiceInfo::setAttribute(quint16 attributeId, const QBluetoothUuid &value)
{
    setAttribute(attributeId, QVariant::fromValue(value));
}

inline void QBluetoothServiceInfo::setAttribute(quint16 attributeId, const QBluetoothServiceInfo::Sequence &value)
{
    setAttribute(attributeId, QVariant::fromValue(value));
}

inline void QBluetoothServiceInfo::setAttribute(quint16 attributeId, const QBluetoothServiceInfo::Alternative &value)
{
    setAttribute(attributeId, QVariant::fromValue(value));
}

inline void QBluetoothServiceInfo::setServiceName(const QString &name)
{
    setAttribute(ServiceName, QVariant::fromValue(name));
}

inline QString QBluetoothServiceInfo::serviceName() const
{
    return attribute(ServiceName).toString();
}

inline void QBluetoothServiceInfo::setServiceDescription(const QString &description)
{
    setAttribute(ServiceDescription, QVariant::fromValue(description));
}

inline QString QBluetoothServiceInfo::serviceDescription() const
{
    return attribute(ServiceDescription).toString();
}

inline void QBluetoothServiceInfo::setServiceProvider(const QString &provider)
{
    setAttribute(ServiceProvider, QVariant::fromValue(provider));
}

inline QString QBluetoothServiceInfo::serviceProvider() const
{
    return attribute(ServiceProvider).toString();
}

inline void QBluetoothServiceInfo::setServiceAvailability(quint8 availability)
{
    setAttribute(ServiceAvailability, QVariant::fromValue(availability));
}

inline quint8 QBluetoothServiceInfo::serviceAvailability() const
{
    return attribute(ServiceAvailability).toUInt();
}

inline void QBluetoothServiceInfo::setServiceUuid(const QBluetoothUuid &uuid)
{
    setAttribute(ServiceId, uuid);
}

inline QBluetoothUuid QBluetoothServiceInfo::serviceUuid() const
{
    return attribute(ServiceId).value<QBluetoothUuid>();
}
QT_END_NAMESPACE

#endif
