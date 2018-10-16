/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef CUSTOMAFFECTOR_H
#define CUSTOMAFFECTOR_H

#include <QObject>
#include "qquickparticlesystem_p.h"
#include "qquickparticleextruder_p.h"
#include "qquickparticleaffector_p.h"
#include "qquickdirection_p.h"

QT_BEGIN_NAMESPACE

class QQuickCustomAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(bool relative READ relative WRITE setRelative NOTIFY relativeChanged)
    Q_PROPERTY(QQuickDirection *position READ position WRITE setPosition NOTIFY positionChanged RESET positionReset)
    Q_PROPERTY(QQuickDirection *velocity READ velocity WRITE setVelocity NOTIFY velocityChanged RESET velocityReset)
    Q_PROPERTY(QQuickDirection *acceleration READ acceleration WRITE setAcceleration NOTIFY accelerationChanged RESET accelerationReset)

public:
    explicit QQuickCustomAffector(QQuickItem *parent = 0);
    virtual void affectSystem(qreal dt);

    QQuickDirection * position() const
    {
        return m_position;
    }

    QQuickDirection * velocity() const
    {
        return m_velocity;
    }

    QQuickDirection * acceleration() const
    {
        return m_acceleration;
    }

    void positionReset()
    {
        m_position = &m_nullVector;
    }

    void velocityReset()
    {
        m_velocity = &m_nullVector;
    }

    void accelerationReset()
    {
        m_acceleration = &m_nullVector;
    }

    bool relative() const
    {
        return m_relative;
    }


Q_SIGNALS:
    void affectParticles(QQmlV4Handle particles, qreal dt);

    void positionChanged(QQuickDirection * arg);

    void velocityChanged(QQuickDirection * arg);

    void accelerationChanged(QQuickDirection * arg);

    void relativeChanged(bool arg);

public Q_SLOTS:
    void setPosition(QQuickDirection * arg)
    {
        if (m_position != arg) {
            m_position = arg;
            Q_EMIT positionChanged(arg);
        }
    }

    void setVelocity(QQuickDirection * arg)
    {
        if (m_velocity != arg) {
            m_velocity = arg;
            Q_EMIT velocityChanged(arg);
        }
    }

    void setAcceleration(QQuickDirection * arg)
    {
        if (m_acceleration != arg) {
            m_acceleration = arg;
            Q_EMIT accelerationChanged(arg);
        }
    }

    void setRelative(bool arg)
    {
        if (m_relative != arg) {
            m_relative = arg;
            Q_EMIT relativeChanged(arg);
        }
    }

protected:
    bool isAffectConnected();
    virtual bool affectParticle(QQuickParticleData *d, qreal dt);
private:
    void affectProperties(const QList<QQuickParticleData*> particles, qreal dt);
    QQuickDirection * m_position;
    QQuickDirection * m_velocity;
    QQuickDirection * m_acceleration;

    QQuickDirection m_nullVector;
    bool m_relative;
};

QT_END_NAMESPACE
#endif // CUSTOMAFFECTOR_H
