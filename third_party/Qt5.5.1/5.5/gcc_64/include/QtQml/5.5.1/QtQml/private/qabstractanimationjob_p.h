/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QABSTRACTANIMATIONJOB_P_H
#define QABSTRACTANIMATIONJOB_P_H

#include <private/qtqmlglobal_p.h>
#include <QtCore/QObject>
#include <QtCore/private/qabstractanimation_p.h>
#include "private/qpodvector_p.h"

QT_BEGIN_NAMESPACE

class QAnimationGroupJob;
class QAnimationJobChangeListener;
class Q_QML_PRIVATE_EXPORT QAbstractAnimationJob
{
    Q_DISABLE_COPY(QAbstractAnimationJob)
public:
    enum Direction {
        Forward,
        Backward
    };

    enum State {
        Stopped,
        Paused,
        Running
    };

    QAbstractAnimationJob();
    virtual ~QAbstractAnimationJob();

    //definition
    inline QAnimationGroupJob *group() const {return m_group;}

    inline int loopCount() const {return m_loopCount;}
    void setLoopCount(int loopCount);

    int totalDuration() const;
    virtual int duration() const {return 0;}

    inline QAbstractAnimationJob::Direction direction() const {return m_direction;}
    void setDirection(QAbstractAnimationJob::Direction direction);

    //state
    inline int currentTime() const {return m_totalCurrentTime;}
    inline int currentLoopTime() const {return m_currentTime;}
    inline int currentLoop() const {return m_currentLoop;}
    inline QAbstractAnimationJob::State state() const {return m_state;}
    inline bool isRunning() { return m_state == Running; }
    inline bool isStopped() { return m_state == Stopped; }
    inline bool isPaused() { return m_state == Paused; }
    void setDisableUserControl();
    void setEnableUserControl();
    bool userControlDisabled() const;

    void setCurrentTime(int msecs);

    void start();
    void pause();
    void resume();
    void stop();

    enum ChangeType {
        Completion = 0x01,
        StateChange = 0x02,
        CurrentLoop = 0x04,
        CurrentTime = 0x08
    };
    Q_DECLARE_FLAGS(ChangeTypes, ChangeType)

    void addAnimationChangeListener(QAnimationJobChangeListener *listener, QAbstractAnimationJob::ChangeTypes);
    void removeAnimationChangeListener(QAnimationJobChangeListener *listener, QAbstractAnimationJob::ChangeTypes);
    QAbstractAnimationJob *nextSibling() const { return m_nextSibling; }
    QAbstractAnimationJob *previousSibling() const { return m_previousSibling; }

    bool isGroup() const { return m_isGroup; }
    bool isRenderThreadJob() const { return m_isRenderThreadJob; }
    bool isRenderThreadProxy() const { return m_isRenderThreadProxy; }

protected:
    virtual void updateCurrentTime(int) {}
    virtual void updateState(QAbstractAnimationJob::State newState, QAbstractAnimationJob::State oldState);
    virtual void updateDirection(QAbstractAnimationJob::Direction direction);
    virtual void topLevelAnimationLoopChanged() {}

    virtual void debugAnimation(QDebug d) const;

    void fireTopLevelAnimationLoopChanged();

    void setState(QAbstractAnimationJob::State state);

    void finished();
    void stateChanged(QAbstractAnimationJob::State newState, QAbstractAnimationJob::State oldState);
    void currentLoopChanged();
    void directionChanged(QAbstractAnimationJob::Direction);
    void currentTimeChanged(int currentTime);

    //definition
    int m_loopCount;
    QAnimationGroupJob *m_group;
    QAbstractAnimationJob::Direction m_direction;

    //state
    QAbstractAnimationJob::State m_state;
    int m_totalCurrentTime;
    int m_currentTime;
    int m_currentLoop;
    //records the finish time for an uncontrolled animation (used by animation groups)
    int m_uncontrolledFinishTime;
    int m_currentLoopStartTime; // used together with m_uncontrolledFinishTime

    struct ChangeListener {
        ChangeListener(QAnimationJobChangeListener *l, QAbstractAnimationJob::ChangeTypes t) : listener(l), types(t) {}
        QAnimationJobChangeListener *listener;
        QAbstractAnimationJob::ChangeTypes types;
        bool operator==(const ChangeListener &other) const { return listener == other.listener && types == other.types; }
    };
    QPODVector<ChangeListener,1> changeListeners;

    QAbstractAnimationJob *m_nextSibling;
    QAbstractAnimationJob *m_previousSibling;

    bool *m_wasDeleted;
    bool m_hasRegisteredTimer:1;
    bool m_isPause:1;
    bool m_isGroup:1;
    bool m_disableUserControl:1;
    bool m_hasCurrentTimeChangeListeners:1;
    bool m_isRenderThreadJob:1;
    bool m_isRenderThreadProxy:1;

    friend class QQmlAnimationTimer;
    friend class QAnimationGroupJob;
    friend Q_QML_PRIVATE_EXPORT QDebug operator<<(QDebug, const QAbstractAnimationJob *job);
};

class Q_QML_PRIVATE_EXPORT QAnimationJobChangeListener
{
public:
    virtual ~QAnimationJobChangeListener();
    virtual void animationFinished(QAbstractAnimationJob *) {}
    virtual void animationStateChanged(QAbstractAnimationJob *, QAbstractAnimationJob::State, QAbstractAnimationJob::State) {}
    virtual void animationCurrentLoopChanged(QAbstractAnimationJob *) {}
    virtual void animationCurrentTimeChanged(QAbstractAnimationJob *, int) {}
};

class Q_QML_PRIVATE_EXPORT QQmlAnimationTimer : public QAbstractAnimationTimer
{
    Q_OBJECT
private:
    QQmlAnimationTimer();

public:
    static QQmlAnimationTimer *instance();
    static QQmlAnimationTimer *instance(bool create);

    static void registerAnimation(QAbstractAnimationJob *animation, bool isTopLevel);
    static void unregisterAnimation(QAbstractAnimationJob *animation);

    /*
        this is used for updating the currentTime of all animations in case the pause
        timer is active or, otherwise, only of the animation passed as parameter.
    */
    static void ensureTimerUpdate();

    /*
        this will evaluate the need of restarting the pause timer in case there is still
        some pause animations running.
    */
    static void updateAnimationTimer();

    void restartAnimationTimer();
    void updateAnimationsTime(qint64 timeStep);

    //useful for profiling/debugging
    int runningAnimationCount() { return animations.count(); }

    bool hasStartAnimationPending() const { return startAnimationPending; }

public Q_SLOTS:
    void startAnimations();
    void stopTimer();

private:
    qint64 lastTick;
    int currentAnimationIdx;
    bool insideTick;
    bool startAnimationPending;
    bool stopTimerPending;

    QList<QAbstractAnimationJob*> animations, animationsToStart;

    // this is the count of running animations that are not a group neither a pause animation
    int runningLeafAnimations;
    QList<QAbstractAnimationJob*> runningPauseAnimations;

    void registerRunningAnimation(QAbstractAnimationJob *animation);
    void unregisterRunningAnimation(QAbstractAnimationJob *animation);

    int closestPauseAnimationTimeToFinish();
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QAbstractAnimationJob::ChangeTypes)

Q_QML_PRIVATE_EXPORT QDebug operator<<(QDebug, const QAbstractAnimationJob *job);

QT_END_NAMESPACE

#endif // QABSTRACTANIMATIONJOB_P_H
