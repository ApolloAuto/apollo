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

#ifndef TURBULENCEAFFECTOR_H
#define TURBULENCEAFFECTOR_H
#include "qquickparticleaffector_p.h"
#include <QQmlListProperty>

QT_BEGIN_NAMESPACE

class QQuickParticlePainter;

class QQuickTurbulenceAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(qreal strength READ strength WRITE setStrength NOTIFY strengthChanged)
    Q_PROPERTY(QUrl noiseSource READ noiseSource WRITE setNoiseSource NOTIFY noiseSourceChanged)
    public:
    explicit QQuickTurbulenceAffector(QQuickItem *parent = 0);
    ~QQuickTurbulenceAffector();
    virtual void affectSystem(qreal dt);

    qreal strength() const
    {
        return m_strength;
    }

    QUrl noiseSource() const
    {
        return m_noiseSource;
    }
Q_SIGNALS:

    void strengthChanged(qreal arg);

    void noiseSourceChanged(QUrl arg);

public Q_SLOTS:

    void setStrength(qreal arg)
    {
        if (m_strength != arg) {
            m_strength = arg;
            Q_EMIT strengthChanged(arg);
        }
    }

    void setNoiseSource(QUrl arg)
    {
        if (m_noiseSource != arg) {
            m_noiseSource = arg;
            Q_EMIT noiseSourceChanged(arg);
            initializeGrid();
        }
    }

protected:
    virtual void geometryChanged(const QRectF &newGeometry,
                                 const QRectF &oldGeometry);
private:
    void ensureInit();
    void mapUpdate();
    void initializeGrid();
    qreal boundsRespectingField(int x, int y);
    qreal m_strength;
    qreal m_lastT;
    int m_gridSize;
    qreal** m_field;
    QPointF** m_vectorField;
    bool m_inited;
    QUrl m_noiseSource;
};

QT_END_NAMESPACE
#endif // TURBULENCEAFFECTOR_H
