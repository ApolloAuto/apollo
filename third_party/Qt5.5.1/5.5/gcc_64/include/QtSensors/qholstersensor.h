/****************************************************************************
**
** Copyright (C) 2013 Research In Motion
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
#ifndef QHOLSTERSENSOR_H
#define QHOLSTERSENSOR_H

#include <QtSensors/qsensor.h>

QT_BEGIN_NAMESPACE

class QHolsterReadingPrivate;

class Q_SENSORS_EXPORT QHolsterReading : public QSensorReading
{
    Q_OBJECT
    Q_PROPERTY(bool holstered READ holstered)
    DECLARE_READING(QHolsterReading)
public:
    bool holstered() const;
    void setHolstered(bool holstered);
};

class Q_SENSORS_EXPORT QHolsterFilter : public QSensorFilter
{
public:
    virtual bool filter(QHolsterReading *reading) = 0;
private:
    bool filter(QSensorReading *reading) Q_DECL_OVERRIDE;
};

class Q_SENSORS_EXPORT QHolsterSensor : public QSensor
{
    Q_OBJECT
public:
    explicit QHolsterSensor(QObject *parent = 0);
    ~QHolsterSensor();
    QHolsterReading *reading() const;
    static char const * const type;

private:
    Q_DISABLE_COPY(QHolsterSensor)
};

QT_END_NAMESPACE

#endif
