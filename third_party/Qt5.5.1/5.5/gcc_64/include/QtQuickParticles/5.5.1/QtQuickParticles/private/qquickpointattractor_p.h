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

#ifndef ATTRACTORAFFECTOR_H
#define ATTRACTORAFFECTOR_H
#include "qquickparticleaffector_p.h"

QT_BEGIN_NAMESPACE

class QQuickAttractorAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(qreal strength READ strength WRITE setStrength NOTIFY strengthChanged)
    Q_PROPERTY(qreal pointX READ pointX WRITE setPointX NOTIFY pointXChanged)
    Q_PROPERTY(qreal pointY READ pointY WRITE setPointY NOTIFY pointYChanged)
    Q_PROPERTY(AffectableParameters affectedParameter READ affectedParameter WRITE setAffectedParameter NOTIFY affectedParameterChanged)
    Q_PROPERTY(Proportion proportionalToDistance READ proportionalToDistance WRITE setProportionalToDistance NOTIFY proportionalToDistanceChanged)
    Q_ENUMS(AffectableParameters)
    Q_ENUMS(Proportion)

public:
    enum Proportion{
        Constant,
        Linear,
        Quadratic,
        InverseLinear,
        InverseQuadratic
    };

    enum AffectableParameters {
        Position,
        Velocity,
        Acceleration
    };

    explicit QQuickAttractorAffector(QQuickItem *parent = 0);

    qreal strength() const
    {
        return m_strength;
    }

    qreal pointX() const
    {
        return m_x;
    }

    qreal pointY() const
    {
        return m_y;
    }

    AffectableParameters affectedParameter() const
    {
        return m_physics;
    }

    Proportion proportionalToDistance() const
    {
        return m_proportionalToDistance;
    }

Q_SIGNALS:

    void strengthChanged(qreal arg);

    void pointXChanged(qreal arg);

    void pointYChanged(qreal arg);

    void affectedParameterChanged(AffectableParameters arg);

    void proportionalToDistanceChanged(Proportion arg);

public Q_SLOTS:
void setStrength(qreal arg)
{
    if (m_strength != arg) {
        m_strength = arg;
        Q_EMIT strengthChanged(arg);
    }
}

void setPointX(qreal arg)
{
    if (m_x != arg) {
        m_x = arg;
        Q_EMIT pointXChanged(arg);
    }
}

void setPointY(qreal arg)
{
    if (m_y != arg) {
        m_y = arg;
        Q_EMIT pointYChanged(arg);
    }
}
void setAffectedParameter(AffectableParameters arg)
{
    if (m_physics != arg) {
        m_physics = arg;
        Q_EMIT affectedParameterChanged(arg);
    }
}

void setProportionalToDistance(Proportion arg)
{
    if (m_proportionalToDistance != arg) {
        m_proportionalToDistance = arg;
        Q_EMIT proportionalToDistanceChanged(arg);
    }
}

protected:
    virtual bool affectParticle(QQuickParticleData *d, qreal dt);
private:
qreal m_strength;
qreal m_x;
qreal m_y;
AffectableParameters m_physics;
Proportion m_proportionalToDistance;
};

QT_END_NAMESPACE
#endif // ATTRACTORAFFECTOR_H
