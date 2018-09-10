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

#ifndef WANDERAFFECTOR_H
#define WANDERAFFECTOR_H
#include <QHash>
#include "qquickparticleaffector_p.h"

QT_BEGIN_NAMESPACE

struct WanderData{
    qreal x_vel;
    qreal y_vel;
    qreal x_peak;
    qreal x_var;
    qreal y_peak;
    qreal y_var;
};

class QQuickWanderAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(qreal pace READ pace WRITE setPace NOTIFY paceChanged)
    Q_PROPERTY(qreal xVariance READ xVariance WRITE setXVariance NOTIFY xVarianceChanged)
    Q_PROPERTY(qreal yVariance READ yVariance WRITE setYVariance NOTIFY yVarianceChanged)
    Q_PROPERTY(AffectableParameters affectedParameter READ affectedParameter WRITE setAffectedParameter NOTIFY affectedParameterChanged)
    Q_ENUMS(AffectableParameters)

public:
    enum AffectableParameters {
        Position,
        Velocity,
        Acceleration
    };

    explicit QQuickWanderAffector(QQuickItem *parent = 0);
    ~QQuickWanderAffector();
//    virtual void reset(int systemIdx);

    qreal xVariance() const
    {
        return m_xVariance;
    }

    qreal yVariance() const
    {
        return m_yVariance;
    }

    qreal pace() const
    {
        return m_pace;
    }

    AffectableParameters affectedParameter() const
    {
        return m_affectedParameter;
    }

protected:
    virtual bool affectParticle(QQuickParticleData *d, qreal dt);
Q_SIGNALS:

    void xVarianceChanged(qreal arg);

    void yVarianceChanged(qreal arg);

    void paceChanged(qreal arg);


    void affectedParameterChanged(AffectableParameters arg);

public Q_SLOTS:
void setXVariance(qreal arg)
{
    if (m_xVariance != arg) {
        m_xVariance = arg;
        Q_EMIT xVarianceChanged(arg);
    }
}

void setYVariance(qreal arg)
{
    if (m_yVariance != arg) {
        m_yVariance = arg;
        Q_EMIT yVarianceChanged(arg);
    }
}

void setPace(qreal arg)
{
    if (m_pace != arg) {
        m_pace = arg;
        Q_EMIT paceChanged(arg);
    }
}


void setAffectedParameter(AffectableParameters arg)
{
    if (m_affectedParameter != arg) {
        m_affectedParameter = arg;
        Q_EMIT affectedParameterChanged(arg);
    }
}

private:
    WanderData* getData(int idx);
    QHash<int, WanderData*> m_wanderData;
    qreal m_xVariance;
    qreal m_yVariance;
    qreal m_pace;
    AffectableParameters m_affectedParameter;
};

QT_END_NAMESPACE
#endif // WANDERAFFECTOR_H
