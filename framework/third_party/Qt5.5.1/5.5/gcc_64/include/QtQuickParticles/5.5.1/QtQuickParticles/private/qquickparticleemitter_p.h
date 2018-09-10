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

#ifndef PARTICLEEMITTER_H
#define PARTICLEEMITTER_H

#include <QtQuick/QQuickItem>
#include <QDebug>
#include "qquickparticlesystem_p.h"
#include "qquickparticleextruder_p.h"
#include "qquickdirection_p.h"

#include <QList>
#include <QPair>
#include <QPointF>
QT_BEGIN_NAMESPACE

class QQuickParticleEmitter : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QQuickParticleSystem* system READ system WRITE setSystem NOTIFY systemChanged)
    Q_PROPERTY(QString group READ group WRITE setGroup NOTIFY groupChanged)
    Q_PROPERTY(QQuickParticleExtruder* shape READ extruder WRITE setExtruder NOTIFY extruderChanged)
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(int startTime READ startTime WRITE setStartTime NOTIFY startTimeChanged)

    Q_PROPERTY(qreal emitRate READ particlesPerSecond WRITE setParticlesPerSecond NOTIFY particlesPerSecondChanged)
    Q_PROPERTY(int lifeSpan READ particleDuration WRITE setParticleDuration NOTIFY particleDurationChanged)
    Q_PROPERTY(int lifeSpanVariation READ particleDurationVariation WRITE setParticleDurationVariation NOTIFY particleDurationVariationChanged)
    Q_PROPERTY(int maximumEmitted READ maxParticleCount WRITE setMaxParticleCount NOTIFY maximumEmittedChanged)

    Q_PROPERTY(qreal size READ particleSize WRITE setParticleSize NOTIFY particleSizeChanged)
    Q_PROPERTY(qreal endSize READ particleEndSize WRITE setParticleEndSize NOTIFY particleEndSizeChanged)
    Q_PROPERTY(qreal sizeVariation READ particleSizeVariation WRITE setParticleSizeVariation NOTIFY particleSizeVariationChanged)

    Q_PROPERTY(QQuickDirection *velocity READ velocity WRITE setVelocity NOTIFY velocityChanged)
    Q_PROPERTY(QQuickDirection *acceleration READ acceleration WRITE setAcceleration NOTIFY accelerationChanged)
    Q_PROPERTY(qreal velocityFromMovement READ velocityFromMovement WRITE setVelocityFromMovement NOTIFY velocityFromMovementChanged)

    Q_ENUMS(Lifetime)
public:
    explicit QQuickParticleEmitter(QQuickItem *parent = 0);
    virtual ~QQuickParticleEmitter();
    virtual void emitWindow(int timeStamp);

    enum Lifetime {
        InfiniteLife = QQuickParticleSystem::maxLife
    };

    bool enabled() const
    {
        return m_enabled;
    }

    qreal particlesPerSecond() const
    {
        return m_particlesPerSecond;
    }

    int particleDuration() const
    {
        return m_particleDuration;
    }

    QQuickParticleSystem* system() const
    {
        return m_system;
    }

    QString group() const
    {
        return m_group;
    }

    int particleDurationVariation() const
    {
        return m_particleDurationVariation;
    }

    qreal velocityFromMovement() const { return m_velocity_from_movement; }
    void setVelocityFromMovement(qreal s);
    virtual void componentComplete();
Q_SIGNALS:
    void emitParticles(QQmlV4Handle particles);
    void particlesPerSecondChanged(qreal);
    void particleDurationChanged(int);
    void enabledChanged(bool);

    void systemChanged(QQuickParticleSystem* arg);

    void groupChanged(const QString &arg);

    void particleDurationVariationChanged(int arg);

    void extruderChanged(QQuickParticleExtruder* arg);

    void particleSizeChanged(qreal arg);

    void particleEndSizeChanged(qreal arg);

    void particleSizeVariationChanged(qreal arg);

    void velocityChanged(QQuickDirection * arg);

    void accelerationChanged(QQuickDirection * arg);

    void maximumEmittedChanged(int arg);
    void particleCountChanged();

    void velocityFromMovementChanged();

    void startTimeChanged(int arg);

public Q_SLOTS:
    void pulse(int milliseconds);
    void burst(int num);
    void burst(int num, qreal x, qreal y);

    void setEnabled(bool arg);

    void setParticlesPerSecond(qreal arg)
    {
        if (m_particlesPerSecond != arg) {
            m_particlesPerSecond = arg;
            Q_EMIT particlesPerSecondChanged(arg);
        }
    }

    void setParticleDuration(int arg)
    {
        if (m_particleDuration != arg) {
            m_particleDuration = arg;
            Q_EMIT particleDurationChanged(arg);
        }
    }

    void setSystem(QQuickParticleSystem* arg)
    {
        if (m_system != arg) {
            m_system = arg;
            if (m_system)
                m_system->registerParticleEmitter(this);
            Q_EMIT systemChanged(arg);
        }
    }

    void setGroup(const QString &arg)
    {
        if (m_group != arg) {
            m_group = arg;
            Q_EMIT groupChanged(arg);
        }
    }

    void setParticleDurationVariation(int arg)
    {
        if (m_particleDurationVariation != arg) {
            m_particleDurationVariation = arg;
            Q_EMIT particleDurationVariationChanged(arg);
        }
    }
    void setExtruder(QQuickParticleExtruder* arg)
    {
        if (m_extruder != arg) {
            m_extruder = arg;
            Q_EMIT extruderChanged(arg);
        }
    }

    void setParticleSize(qreal arg)
    {
        if (m_particleSize != arg) {
            m_particleSize = arg;
            Q_EMIT particleSizeChanged(arg);
        }
    }

    void setParticleEndSize(qreal arg)
    {
        if (m_particleEndSize != arg) {
            m_particleEndSize = arg;
            Q_EMIT particleEndSizeChanged(arg);
        }
    }

    void setParticleSizeVariation(qreal arg)
    {
        if (m_particleSizeVariation != arg) {
            m_particleSizeVariation = arg;
            Q_EMIT particleSizeVariationChanged(arg);
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

    void setMaxParticleCount(int arg);

    void setStartTime(int arg)
    {
        if (m_startTime != arg) {
            m_startTime = arg;
            Q_EMIT startTimeChanged(arg);
        }
    }

       virtual void reset();
public:
       int particleCount() const;

       QQuickParticleExtruder* extruder() const
       {
           return m_extruder;
       }

       qreal particleSize() const
       {
           return m_particleSize;
       }

       qreal particleEndSize() const
       {
           return m_particleEndSize;
       }

       qreal particleSizeVariation() const
       {
           return m_particleSizeVariation;
       }

       QQuickDirection * velocity() const
       {
           return m_velocity;
       }

       QQuickDirection * acceleration() const
       {
           return m_acceleration;
       }

       int maxParticleCount() const
       {
           return m_maxParticleCount;
       }

       int startTime() const
       {
           return m_startTime;
       }

protected:
       qreal m_particlesPerSecond;
       int m_particleDuration;
       int m_particleDurationVariation;
       bool m_enabled;
       QQuickParticleSystem* m_system;
       QString m_group;
       QQuickParticleExtruder* m_extruder;
       QQuickParticleExtruder* m_defaultExtruder;
       QQuickParticleExtruder* effectiveExtruder();
       QQuickDirection * m_velocity;
       QQuickDirection * m_acceleration;
       qreal m_particleSize;
       qreal m_particleEndSize;
       qreal m_particleSizeVariation;

       qreal m_velocityFromMovement;
       int m_startTime;
       bool m_overwrite;

       int m_pulseLeft;
       QList<QPair<int, QPointF > > m_burstQueue;
       int m_maxParticleCount;

       //Used in default implementation, but might be useful
       qreal m_velocity_from_movement;

       int m_emitCap;
       bool m_reset_last;
       qreal m_last_timestamp;
       qreal m_last_emission;

       QPointF m_last_emitter;
       QPointF m_last_last_emitter;
       QPointF m_last_last_last_emitter;

       bool isEmitConnected();
private:
       QQuickDirection m_nullVector;

};

QT_END_NAMESPACE

#endif // PARTICLEEMITTER_H
