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

#ifndef QQUICKITEMVIEWTRANSITION_P_P_H
#define QQUICKITEMVIEWTRANSITION_P_P_H

#include <private/qquicktransitionmanager_p_p.h>

QT_BEGIN_NAMESPACE

class QQuickItem;
class QQuickItemViewTransitionableItem;
class QQuickItemViewTransitionJob;


class QQuickItemViewTransitionChangeListener
{
public:
    QQuickItemViewTransitionChangeListener() {}
    virtual ~QQuickItemViewTransitionChangeListener() {}

    virtual void viewItemTransitionFinished(QQuickItemViewTransitionableItem *item) = 0;
};


class QQuickItemViewTransitioner
{
public:
    enum TransitionType {
        NoTransition,
        PopulateTransition,
        AddTransition,
        MoveTransition,
        RemoveTransition
    };

    QQuickItemViewTransitioner();
    virtual ~QQuickItemViewTransitioner();

    bool canTransition(QQuickItemViewTransitioner::TransitionType type, bool asTarget) const;
    void transitionNextReposition(QQuickItemViewTransitionableItem *item, QQuickItemViewTransitioner::TransitionType type, bool isTarget);

    void addToTargetLists(QQuickItemViewTransitioner::TransitionType type, QQuickItemViewTransitionableItem *item, int index);
    void resetTargetLists();

    QQuickTransition *transitionObject(QQuickItemViewTransitioner::TransitionType type, bool asTarget);
    const QList<int> &targetIndexes(QQuickItemViewTransitioner::TransitionType type) const;
    const QList<QObject *> &targetItems(QQuickItemViewTransitioner::TransitionType type) const;

    inline void setPopulateTransitionEnabled(bool b) { usePopulateTransition = b; }
    inline bool populateTransitionEnabled() const { return usePopulateTransition; }

    inline void setChangeListener(QQuickItemViewTransitionChangeListener *obj) { changeListener = obj; }

    QSet<QQuickItemViewTransitionJob *> runningJobs;

    QList<int> addTransitionIndexes;
    QList<int> moveTransitionIndexes;
    QList<int> removeTransitionIndexes;
    QList<QObject *> addTransitionTargets;
    QList<QObject *> moveTransitionTargets;
    QList<QObject *> removeTransitionTargets;

    QQuickTransition *populateTransition;
    QQuickTransition *addTransition;
    QQuickTransition *addDisplacedTransition;
    QQuickTransition *moveTransition;
    QQuickTransition *moveDisplacedTransition;
    QQuickTransition *removeTransition;
    QQuickTransition *removeDisplacedTransition;
    QQuickTransition *displacedTransition;

private:
    friend class QQuickItemViewTransitionJob;

    QQuickItemViewTransitionChangeListener *changeListener;
    bool usePopulateTransition;

    void finishedTransition(QQuickItemViewTransitionJob *job, QQuickItemViewTransitionableItem *item);
};


/*
  An item that can be transitioned using QQuickViewTransitionJob.
  */
class QQuickItemViewTransitionableItem
{
public:
    QQuickItemViewTransitionableItem(QQuickItem *i);
    virtual ~QQuickItemViewTransitionableItem();

    qreal itemX() const;
    qreal itemY() const;

    void moveTo(const QPointF &pos, bool immediate = false);

    bool transitionScheduledOrRunning() const;
    bool transitionRunning() const;
    bool isPendingRemoval() const;

    bool prepareTransition(QQuickItemViewTransitioner *transitioner, int index, const QRectF &viewBounds);
    void startTransition(QQuickItemViewTransitioner *transitioner, int index);

    QPointF nextTransitionTo;
    QPointF lastMovedTo;
    QPointF nextTransitionFrom;
    QQuickItem *item;
    QQuickItemViewTransitionJob *transition;
    QQuickItemViewTransitioner::TransitionType nextTransitionType;
    bool isTransitionTarget : 1;
    bool nextTransitionToSet : 1;
    bool nextTransitionFromSet : 1;
    bool lastMovedToSet : 1;
    bool prepared : 1;

private:
    friend class QQuickItemViewTransitioner;
    friend class QQuickItemViewTransitionJob;
    void setNextTransition(QQuickItemViewTransitioner::TransitionType, bool isTargetItem);
    bool transitionWillChangePosition() const;
    void finishedTransition();
    void resetNextTransitionPos();
    void clearCurrentScheduledTransition();
    void stopTransition();
};


class QQuickViewTransitionAttached : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int index READ index NOTIFY indexChanged)
    Q_PROPERTY(QQuickItem* item READ item NOTIFY itemChanged)
    Q_PROPERTY(QPointF destination READ destination NOTIFY destinationChanged)

    Q_PROPERTY(QList<int> targetIndexes READ targetIndexes NOTIFY targetIndexesChanged)
    Q_PROPERTY(QQmlListProperty<QObject> targetItems READ targetItems NOTIFY targetItemsChanged)

public:
    QQuickViewTransitionAttached(QObject *parent);

    int index() const { return m_index; }
    QQuickItem *item() const { return m_item; }
    QPointF destination() const { return m_destination; }

    QList<int> targetIndexes() const { return m_targetIndexes; }
    QQmlListProperty<QObject> targetItems();

    static QQuickViewTransitionAttached *qmlAttachedProperties(QObject *);

Q_SIGNALS:
    void indexChanged();
    void itemChanged();
    void destinationChanged();

    void targetIndexesChanged();
    void targetItemsChanged();

private:
    friend class QQuickItemViewTransitionJob;
    QPointF m_destination;
    QList<int> m_targetIndexes;
    QList<QObject *> m_targetItems;

    QQuickItem *m_item;
    int m_index;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickViewTransitionAttached)
QML_DECLARE_TYPEINFO(QQuickViewTransitionAttached, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQUICKITEMVIEWTRANSITION_P_P_H
