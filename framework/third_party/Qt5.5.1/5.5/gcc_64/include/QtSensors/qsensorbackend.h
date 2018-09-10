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

#ifndef QSENSORBACKEND_H
#define QSENSORBACKEND_H

#include <QtSensors/qsensor.h>
#include <QtSensors/qsensormanager.h>

QT_BEGIN_NAMESPACE

class QSensorBackendPrivate;

class Q_SENSORS_EXPORT QSensorBackend : public QObject
{
    Q_OBJECT
public:
    explicit QSensorBackend(QSensor *sensor, QObject *parent = 0);
    virtual ~QSensorBackend();

    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool isFeatureSupported(QSensor::Feature feature) const;

    // used by the backend to set metadata properties
    void addDataRate(qreal min, qreal max);
    void setDataRates(const QSensor *otherSensor);
    void addOutputRange(qreal min, qreal max, qreal accuracy);
    void setDescription(const QString &description);

    template <typename T>
    T *setReading(T *readingClass)
    {
        if (!readingClass)
            readingClass = new T(this);
        setReadings(readingClass, new T(this), new T(this));
        return readingClass;
    }

    QSensorReading *reading() const;
    QSensor *sensor() const;

    // used by the backend to inform us of events
    void newReadingAvailable();
    void sensorStopped();
    void sensorBusy();
    void sensorError(int error);

private:
    void setReadings(QSensorReading *device, QSensorReading *filter, QSensorReading *cache);

    Q_DECLARE_PRIVATE(QSensorBackend)
    Q_DISABLE_COPY(QSensorBackend)
};

QT_END_NAMESPACE

#endif

