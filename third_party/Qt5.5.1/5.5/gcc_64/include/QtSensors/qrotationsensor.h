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

#ifndef QROTATIONSENSOR_H
#define QROTATIONSENSOR_H

#include <QtSensors/qsensor.h>

QT_BEGIN_NAMESPACE

class QRotationReadingPrivate;

class Q_SENSORS_EXPORT QRotationReading : public QSensorReading
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x)
    Q_PROPERTY(qreal y READ y)
    Q_PROPERTY(qreal z READ z)
    DECLARE_READING(QRotationReading)
public:
    qreal x() const;
    qreal y() const;
    qreal z() const;

    void setFromEuler(qreal x, qreal y, qreal z);
};

class Q_SENSORS_EXPORT QRotationFilter : public QSensorFilter
{
public:
    virtual bool filter(QRotationReading *reading) = 0;
private:
    bool filter(QSensorReading *reading) Q_DECL_OVERRIDE;
};

class QRotationSensorPrivate;

class Q_SENSORS_EXPORT QRotationSensor : public QSensor
{
    Q_OBJECT
    Q_PROPERTY(bool hasZ READ hasZ NOTIFY hasZChanged)
public:
    explicit QRotationSensor(QObject *parent = 0);
    virtual ~QRotationSensor();
    QRotationReading *reading() const;
    static char const * const type;

    bool hasZ() const;
    void setHasZ(bool hasZ);

Q_SIGNALS:
    void hasZChanged(bool hasZ);

private:
    Q_DECLARE_PRIVATE(QRotationSensor)
    Q_DISABLE_COPY(QRotationSensor)
};

QT_END_NAMESPACE

#endif

