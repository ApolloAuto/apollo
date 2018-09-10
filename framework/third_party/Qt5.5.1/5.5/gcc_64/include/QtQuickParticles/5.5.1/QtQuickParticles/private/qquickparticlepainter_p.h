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

#ifndef PARTICLE_H
#define PARTICLE_H

#include <QObject>
#include <QDebug>
#include <QPair>
#include "qquickparticlesystem_p.h"

QT_BEGIN_NAMESPACE

class QQuickParticlePainter : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QQuickParticleSystem* system READ system WRITE setSystem NOTIFY systemChanged)
    Q_PROPERTY(QStringList groups READ groups WRITE setGroups NOTIFY groupsChanged)

public:
    explicit QQuickParticlePainter(QQuickItem *parent = 0);
    //Data Interface to system
    void load(QQuickParticleData*);
    void reload(QQuickParticleData*);
    void setCount(int c);
    int count();
    void performPendingCommits();//Called from updatePaintNode
    QQuickParticleSystem* system() const
    {
        return m_system;
    }


    QStringList groups() const
    {
        return m_groups;
    }

    void itemChange(ItemChange, const ItemChangeData &);

Q_SIGNALS:
    void countChanged();
    void systemChanged(QQuickParticleSystem* arg);

    void groupsChanged(const QStringList &arg);

public Q_SLOTS:
    void setSystem(QQuickParticleSystem* arg);

    void setGroups(const QStringList &arg)
    {
        if (m_groups != arg) {
            m_groups = arg;
            //Note: The system watches this as it has to recalc things when groups change. It will request a reset if necessary
            Q_EMIT groupsChanged(arg);
        }
    }

    void calcSystemOffset(bool resetPending = false);

private Q_SLOTS:
    virtual void sceneGraphInvalidated() {}

protected:
    /* Reset resets all your internal data structures. But anything attached to a particle should
       be in attached data. So reset + reloads should have no visible effect.
       ###Hunt down all cases where we do a complete reset for convenience and be more targeted
    */
    virtual void reset();

    virtual void componentComplete();
    virtual void initialize(int gIdx, int pIdx){//Called from main thread
        Q_UNUSED(gIdx);
        Q_UNUSED(pIdx);
    }
    virtual void commit(int gIdx, int pIdx){//Called in Render Thread
        //###If you need to do something on size changed, check m_data size in this? Or we reset you every time?
        Q_UNUSED(gIdx);
        Q_UNUSED(pIdx);
    }

    QQuickParticleSystem* m_system;
    friend class QQuickParticleSystem;
    int m_count;
    bool m_pleaseReset;//Used by subclasses, but it's a nice optimization to know when stuff isn't going to matter.
    QStringList m_groups;
    QPointF m_systemOffset;

    QQuickWindow *m_window;

private:
    QSet<QPair<int,int> > m_pendingCommits;
};

QT_END_NAMESPACE
#endif // PARTICLE_H
