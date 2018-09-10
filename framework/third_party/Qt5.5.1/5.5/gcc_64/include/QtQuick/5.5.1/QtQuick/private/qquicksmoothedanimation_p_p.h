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

#ifndef QQUICKSMOOTHEDANIMATION2_P_H
#define QQUICKSMOOTHEDANIMATION2_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include "qquicksmoothedanimation_p.h"
#include "qquickanimation_p.h"

#include "qquickanimation_p_p.h"

#include <private/qobject_p.h>
#include <QBasicTimer>

QT_BEGIN_NAMESPACE
class QSmoothedAnimation;
class QSmoothedAnimationTimer : public QTimer
{
    Q_OBJECT
public:
    explicit QSmoothedAnimationTimer(QSmoothedAnimation *animation, QObject *parent = 0);
    ~QSmoothedAnimationTimer();
public Q_SLOTS:
    void stopAnimation();
private:
    QSmoothedAnimation *m_animation;
};

class QQuickSmoothedAnimationPrivate;
class Q_AUTOTEST_EXPORT QSmoothedAnimation : public QAbstractAnimationJob
{
    Q_DISABLE_COPY(QSmoothedAnimation)
public:
    QSmoothedAnimation(QQuickSmoothedAnimationPrivate * = 0);

    ~QSmoothedAnimation();
    qreal to;
    qreal velocity;
    int userDuration;

    int maximumEasingTime;
    QQuickSmoothedAnimation::ReversingMode reversingMode;

    qreal initialVelocity;
    qreal trackVelocity;

    QQmlProperty target;

    int duration() const;
    void restart();
    void init();

    void prepareForRestart();
    void clearTemplate() { animationTemplate = 0; }

protected:
    virtual void updateCurrentTime(int);
    virtual void updateState(QAbstractAnimationJob::State, QAbstractAnimationJob::State);
    void debugAnimation(QDebug d) const;

private:
    qreal easeFollow(qreal);
    qreal initialValue;

    bool invert;

    int finalDuration;

    // Parameters for use in updateCurrentTime()
    qreal a;  // Acceleration
    qreal d;  // Deceleration
    qreal tf; // Total time
    qreal tp; // Time at which peak velocity occurs
    qreal td; // Time at which deceleration begins
    qreal vp; // Velocity at tp
    qreal sp; // Displacement at tp
    qreal sd; // Displacement at td
    qreal vi; // "Normalized" initialvelocity
    qreal s;  // Total s

    int lastTime;
    bool skipUpdate;

    bool recalc();
    void delayedStop();
    QSmoothedAnimationTimer *delayedStopTimer;
    QQuickSmoothedAnimationPrivate *animationTemplate;
};

class QQuickSmoothedAnimationPrivate : public QQuickPropertyAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickSmoothedAnimation)
public:
    QQuickSmoothedAnimationPrivate();
    ~QQuickSmoothedAnimationPrivate();
    void updateRunningAnimations();

    QSmoothedAnimation *anim;
    QHash<QQmlProperty, QSmoothedAnimation*> activeAnimations;
};

QT_END_NAMESPACE

#endif // QQUICKSMOOTHEDANIMATION2_P_H
