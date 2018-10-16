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

#ifndef SPRITEGOALAFFECTOR_H
#define SPRITEGOALAFFECTOR_H
#include "qquickparticleaffector_p.h"
#include <QtQml/qqmlinfo.h>

QT_BEGIN_NAMESPACE

class QQuickStochasticEngine;

class QQuickSpriteGoalAffector : public QQuickParticleAffector
{
    Q_OBJECT
    Q_PROPERTY(QString goalState READ goalState WRITE setGoalState NOTIFY goalStateChanged)
    Q_PROPERTY(bool jump READ jump WRITE setJump NOTIFY jumpChanged)
    Q_PROPERTY(bool systemStates READ systemStates WRITE setSystemStates NOTIFY systemStatesChanged)
public:
    explicit QQuickSpriteGoalAffector(QQuickItem *parent = 0);

    QString goalState() const
    {
        return m_goalState;
    }

    bool jump() const
    {
        return m_jump;
    }
    bool systemStates() const
    {
        return m_systemStates;
    }

protected:
    virtual bool affectParticle(QQuickParticleData *d, qreal dt);
Q_SIGNALS:

    void goalStateChanged(const QString &arg);

    void jumpChanged(bool arg);

    void systemStatesChanged(bool arg);

public Q_SLOTS:

void setGoalState(const QString &arg);

void setJump(bool arg)
{
    if (m_jump != arg) {
        m_jump = arg;
        Q_EMIT jumpChanged(arg);
    }
}

void setSystemStates(bool arg)
{
    if (m_systemStates != arg) {
        //TODO: GroupGoal was added (and this deprecated) Oct 4 - remove it in a few weeks.
        qmlInfo(this) << "systemStates is deprecated and will be removed soon. Use GroupGoal instead.";
        m_systemStates = arg;
        Q_EMIT systemStatesChanged(arg);
    }
}

private:
    void updateStateIndex(QQuickStochasticEngine* e);
    QString m_goalState;
    int m_goalIdx;
    QQuickStochasticEngine* m_lastEngine;
    bool m_jump;
    bool m_systemStates;

    bool m_notUsingEngine;
};

QT_END_NAMESPACE

#endif // SPRITEGOALAFFECTOR_H
