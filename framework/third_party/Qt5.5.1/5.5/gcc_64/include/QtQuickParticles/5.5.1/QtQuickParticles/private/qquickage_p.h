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

#ifndef KILLAFFECTOR_H
#define KILLAFFECTOR_H
#include "qquickparticleaffector_p.h"

QT_BEGIN_NAMESPACE

class QQuickAgeAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(int lifeLeft READ lifeLeft WRITE setLifeLeft NOTIFY lifeLeftChanged)
    Q_PROPERTY(bool advancePosition READ advancePosition WRITE setAdvancePosition NOTIFY advancePositionChanged)

public:
    explicit QQuickAgeAffector(QQuickItem *parent = 0);

    int lifeLeft() const
    {
        return m_lifeLeft;
    }

    bool advancePosition() const
    {
        return m_advancePosition;
    }

protected:
    virtual bool affectParticle(QQuickParticleData *d, qreal dt);
Q_SIGNALS:
    void lifeLeftChanged(int arg);
    void advancePositionChanged(bool arg);

public Q_SLOTS:
    void setLifeLeft(int arg)
    {
        if (m_lifeLeft != arg) {
            m_lifeLeft = arg;
            Q_EMIT lifeLeftChanged(arg);
        }
    }

    void setAdvancePosition(bool arg)
    {
        if (m_advancePosition != arg) {
            m_advancePosition = arg;
            Q_EMIT advancePositionChanged(arg);
        }
    }

private:
    int m_lifeLeft;
    bool m_advancePosition;
};

QT_END_NAMESPACE
#endif // KILLAFFECTOR_H
