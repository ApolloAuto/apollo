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

#ifndef LOWENERGYNOTIFICATIONHUB_H
#define LOWENERGYNOTIFICATIONHUB_H

#include <QtCore/QObject>
#include <QtCore/QReadWriteLock>
#include <QtCore/private/qjnihelpers_p.h>
#include <QtAndroidExtras/QAndroidJniObject>
#include <QtBluetooth/QBluetoothAddress>
#include <jni.h>

#include <QtBluetooth/QLowEnergyCharacteristic>
#include "qlowenergycontroller_p.h"

QT_BEGIN_NAMESPACE

class LowEnergyNotificationHub : public QObject
{
    Q_OBJECT
public:
    explicit LowEnergyNotificationHub(const QBluetoothAddress &remote,
                                      QObject *parent = 0);
    ~LowEnergyNotificationHub();

    static void lowEnergy_connectionChange(JNIEnv*, jobject, jlong qtObject,
                                           jint errorCode, jint newState);
    static void lowEnergy_servicesDiscovered(JNIEnv*, jobject, jlong qtObject,
                                             jint errorCode, jobject uuidList);
    static void lowEnergy_serviceDetailsDiscovered(JNIEnv *, jobject,
                                                   jlong qtObject, jobject uuid,
                                                   jint startHandle, jint endHandle);
    static void lowEnergy_characteristicRead(JNIEnv*env, jobject, jlong qtObject,
                                             jobject serviceUuid,
                                             jint handle, jobject charUuid,
                                             jint properties, jbyteArray data);
    static void lowEnergy_descriptorRead(JNIEnv *env, jobject, jlong qtObject,
                                         jobject sUuid, jobject cUuid,
                                         jint handle, jobject dUuid, jbyteArray data);
    static void lowEnergy_characteristicWritten(JNIEnv *, jobject, jlong qtObject,
                                                jint charHandle, jbyteArray data,
                                                jint errorCode);
    static void lowEnergy_descriptorWritten(JNIEnv *, jobject, jlong qtObject,
                                            jint descHandle, jbyteArray data,
                                            jint errorCode);
    static void lowEnergy_characteristicChanged(JNIEnv *, jobject, jlong qtObject,
                                                jint charHandle, jbyteArray data);
    static void lowEnergy_serviceError(JNIEnv *, jobject, jlong qtObject,
                                       jint attributeHandle, int errorCode);

    QAndroidJniObject javaObject()
    {
        return jBluetoothLe;
    }

signals:
    void connectionUpdated(QLowEnergyController::ControllerState newState,
            QLowEnergyController::Error errorCode);
    void servicesDiscovered(QLowEnergyController::Error errorCode, const QString &uuids);
    void serviceDetailsDiscoveryFinished(const QString& serviceUuid,
            int startHandle, int endHandle);
    void characteristicRead(const QBluetoothUuid &serviceUuid,
            int handle, const QBluetoothUuid &charUuid,
            int properties, const QByteArray &data);
    void descriptorRead(const QBluetoothUuid &serviceUuid, const QBluetoothUuid &charUuid,
            int handle, const QBluetoothUuid &descUuid, const QByteArray &data);
    void characteristicWritten(int charHandle, const QByteArray &data,
                               QLowEnergyService::ServiceError errorCode);
    void descriptorWritten(int descHandle, const QByteArray &data,
                           QLowEnergyService::ServiceError errorCode);
    void characteristicChanged(int charHandle, const QByteArray &data);
    void serviceError(int attributeHandle, QLowEnergyService::ServiceError errorCode);

public slots:
private:
    static QReadWriteLock lock;

    QAndroidJniObject jBluetoothLe;
    long javaToCtoken;

};

QT_END_NAMESPACE

#endif // LOWENERGYNOTIFICATIONHUB_H

