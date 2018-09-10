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

#ifndef QQUICKANIMATORCONTROLLER_P_H
#define QQUICKANIMATORCONTROLLER_P_H

#include "qquickanimatorjob_p.h"
#include <QtQuick/qsgnode.h>
#include <QtQuick/qquickitem.h>

#include <QtCore/qmutex.h>
#include <QtCore/qthread.h>

QT_BEGIN_NAMESPACE

class QQuickAnimatorControllerGuiThreadEntity;

class QQuickAnimatorController : public QObject, public QAnimationJobChangeListener
{
    Q_OBJECT

public:
    QQuickAnimatorController(QQuickWindow *window);
    ~QQuickAnimatorController();

    void advance();
    void beforeNodeSync();
    void afterNodeSync();

    void animationFinished(QAbstractAnimationJob *job);
    void animationStateChanged(QAbstractAnimationJob *job, QAbstractAnimationJob::State newState, QAbstractAnimationJob::State oldState);

    void requestSync();

    // These are called from the GUI thread (the proxy)
    void startJob(QQuickAnimatorProxyJob *proxy, QAbstractAnimationJob *job);
    void stopJob(QQuickAnimatorProxyJob *proxy, QAbstractAnimationJob *job);
    void deleteJob(QAbstractAnimationJob *job);

    void lock() { m_mutex.lock(); }
    void unlock() { m_mutex.unlock(); }


    void proxyWasDestroyed(QQuickAnimatorProxyJob *proxy);
    void stopProxyJobs();
    void windowNodesDestroyed();

public Q_SLOTS:
    void itemDestroyed(QObject *);

public:
    // These are manipulated from the GUI thread and should only
    // be updated during the sync() phase.
    QHash<QAbstractAnimationJob *, QQuickAnimatorProxyJob *> m_starting;
    QHash<QAbstractAnimationJob *, QQuickAnimatorProxyJob *> m_stopping;
    QSet<QAbstractAnimationJob *> m_deleting;

    QHash<QAbstractAnimationJob *, QQuickAnimatorProxyJob *> m_animatorRoots;
    QSet<QQuickAnimatorJob *> m_activeLeafAnimations;
    QHash<QQuickItem *, QQuickTransformAnimatorJob::Helper *> m_transforms;
    QSet<QQuickItem *> m_deletedSinceLastFrame;
    QQuickWindow *m_window;
    QQuickAnimatorControllerGuiThreadEntity *m_guiEntity;
    QSet<QQuickAnimatorProxyJob *> m_proxiesToStop;
    QMutex m_mutex;

    bool m_nodesAreInvalid;
};

class QQuickAnimatorControllerGuiThreadEntity : public QObject
{
    Q_OBJECT
public:
    QPointer<QQuickAnimatorController> controller;

public Q_SLOTS:
    void frameSwapped();
};



QT_END_NAMESPACE

#endif // QQUICKANIMATORCONTROLLER_P_H
