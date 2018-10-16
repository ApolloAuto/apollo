/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtSensors module of the Qt Toolkit.
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

#ifndef QACCELEROMETER_H
#define QACCELEROMETER_H

#include <QtSensors/qsensor.h>

QT_BEGIN_NAMESPACE

class QAccelerometerReadingPrivate;

class Q_SENSORS_EXPORT QAccelerometerReading : public QSensorReading
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x)
    Q_PROPERTY(qreal y READ y)
    Q_PROPERTY(qreal z READ z)
    DECLARE_READING(QAccelerometerReading)
public:
    qreal x() const;
    void setX(qreal x);

    qreal y() const;
    void setY(qreal y);

    qreal z() const;
    void setZ(qreal z);
};

class Q_SENSORS_EXPORT QAccelerometerFilter : public QSensorFilter
{
public:
    virtual bool filter(QAccelerometerReading *reading) = 0;
private:
    bool filter(QSensorReading *reading);
};

class QAccelerometerPrivate;

class Q_SENSORS_EXPORT QAccelerometer : public QSensor
{
    Q_OBJECT
    Q_ENUMS(AccelerationMode)
    Q_PROPERTY(AccelerationMode accelerationMode READ accelerationMode WRITE setAccelerationMode
               NOTIFY accelerationModeChanged)
public:
    explicit QAccelerometer(QObject *parent = 0);
    virtual ~QAccelerometer();

    // Keep this enum in sync with QmlAccelerometer::AccelerationMode
    enum AccelerationMode {
        Combined,
        Gravity,
        User
    };

    AccelerationMode accelerationMode() const;
    void setAccelerationMode(AccelerationMode accelerationMode);

    QAccelerometerReading *reading() const;
    static char const * const type;

Q_SIGNALS:
    void accelerationModeChanged(AccelerationMode accelerationMode);

private:
    Q_DECLARE_PRIVATE(QAccelerometer)
    Q_DISABLE_COPY(QAccelerometer)
};

QT_END_NAMESPACE

#endif

