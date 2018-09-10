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

#ifndef ITEMPARTICLE_H
#define ITEMPARTICLE_H
#include "qquickparticlepainter_p.h"
#include <QPointer>
#include <QSet>
#include <private/qquickanimation_p_p.h>
QT_BEGIN_NAMESPACE

class QQuickVisualDataModel;
class QQuickItemParticleAttached;

class QQuickItemParticle : public QQuickParticlePainter
{
    Q_OBJECT
    Q_PROPERTY(bool fade READ fade WRITE setFade NOTIFY fadeChanged)
    Q_PROPERTY(QQmlComponent* delegate READ delegate WRITE setDelegate NOTIFY delegateChanged)
public:
    explicit QQuickItemParticle(QQuickItem *parent = 0);
    ~QQuickItemParticle();

    bool fade() const { return m_fade; }

    virtual QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

    static QQuickItemParticleAttached *qmlAttachedProperties(QObject *object);
    QQmlComponent* delegate() const
    {
        return m_delegate;
    }

Q_SIGNALS:
    void fadeChanged();

    void delegateChanged(QQmlComponent* arg);

public Q_SLOTS:
    //TODO: Add a follow mode, where moving the delegate causes the logical particle to go with it?
    void freeze(QQuickItem* item);
    void unfreeze(QQuickItem* item);
    void take(QQuickItem* item,bool prioritize=false);//take by modelparticle
    void give(QQuickItem* item);//give from modelparticle

    void setFade(bool arg){if (arg == m_fade) return; m_fade = arg; Q_EMIT fadeChanged();}
    void setDelegate(QQmlComponent* arg)
    {
        if (m_delegate != arg) {
            m_delegate = arg;
            Q_EMIT delegateChanged(arg);
        }
    }

protected:
    virtual void reset();
    virtual void commit(int gIdx, int pIdx);
    virtual void initialize(int gIdx, int pIdx);
    void prepareNextFrame();
private:
    void processDeletables();
    void tick(int time = 0);
    QList<QQuickItem* > m_deletables;
    QList<QQuickItem* > m_managed;
    QList< QQuickParticleData* > m_loadables;
    bool m_fade;

    QList<QQuickItem*> m_pendingItems;
    QList<int> m_available;
    QSet<QQuickItem*> m_stasis;
    qreal m_lastT;
    int m_activeCount;
    QQmlComponent* m_delegate;

    typedef QTickAnimationProxy<QQuickItemParticle, &QQuickItemParticle::tick> Clock;
    Clock *clock;
};

class QQuickItemParticleAttached : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QQuickItemParticle* particle READ particle CONSTANT);
public:
    QQuickItemParticleAttached(QObject* parent)
        : QObject(parent), m_mp(0)
    {;}
    QQuickItemParticle* particle() {return m_mp;}
    void detach(){Q_EMIT detached();}
    void attach(){Q_EMIT attached();}
private:
    QQuickItemParticle* m_mp;
    friend class QQuickItemParticle;
Q_SIGNALS:
    void detached();
    void attached();
};

QT_END_NAMESPACE

QML_DECLARE_TYPEINFO(QQuickItemParticle, QML_HAS_ATTACHED_PROPERTIES)

#endif // ITEMPARTICLE_H
