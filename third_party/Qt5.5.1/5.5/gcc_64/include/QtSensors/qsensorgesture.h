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

#ifndef QSENSORGESTURE_H
#define QSENSORGESTURE_H

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtSensors/qsensorsglobal.h>

#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtCore/QVector>

#include <QtCore/qmetatype.h>

QT_BEGIN_NAMESPACE

class QSensorGesturePrivate;

class Q_SENSORS_EXPORT QSensorGesture : public QObject
{
    //Do not use Q_OBJECT here
public:
    QSensorGesture(const QStringList &ids, QObject *parent = 0);
    ~QSensorGesture();

    bool isActive();

    QStringList validIds() const;
    QStringList invalidIds() const;

    QStringList gestureSignals() const;

    void startDetection();
    void stopDetection();

private:
    QSensorGesturePrivate * d_ptr;

private:
    // Pretend to be a Q_OBJECT
    const QMetaObject *metaObject() const;
    int qt_metacall(QMetaObject::Call, int, void **);

#ifdef Q_QDOC
Q_SIGNALS:
    // these signals are created at runtime, along with
    // gesture recognizer specific signals.
     void detected(QString);
#endif
};

QT_END_NAMESPACE


#endif // QSENSORGESTURE_H
