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

#ifndef QQUICKANIMATORJOB_P_H
#define QQUICKANIMATORJOB_P_H

#include <private/qabstractanimationjob_p.h>
#include <private/qquickanimator_p.h>
#include <private/qtquickglobal_p.h>

#include <QtQuick/qquickitem.h>

#include <QtCore/qeasingcurve.h>

QT_BEGIN_NAMESPACE

class QQuickAnimator;
class QQuickWindow;
class QQuickItem;
class QQuickAbstractAnimation;

class QQuickAnimatorController;
class QQuickAnimatorProxyJobPrivate;
class QQuickShaderEffectNode;

class QSGOpacityNode;

class Q_QUICK_PRIVATE_EXPORT QQuickAnimatorProxyJob : public QObject, public QAbstractAnimationJob
{
    Q_OBJECT

public:
    QQuickAnimatorProxyJob(QAbstractAnimationJob *job, QObject *item);
    ~QQuickAnimatorProxyJob();

    int duration() const Q_DECL_OVERRIDE { return m_duration; }

    QAbstractAnimationJob *job() const { return m_job; }

    void startedByController();
    void controllerWasDeleted();
    void markJobManagedByController() { m_jobManagedByController = true; }

protected:
    void updateCurrentTime(int) Q_DECL_OVERRIDE;
    void updateState(QAbstractAnimationJob::State newState, QAbstractAnimationJob::State oldState) Q_DECL_OVERRIDE;
    void debugAnimation(QDebug d) const Q_DECL_OVERRIDE;

public Q_SLOTS:
    void windowChanged(QQuickWindow *window);
    void sceneGraphInitialized();

private:
    void deleteJob();
    void syncBackCurrentValues();
    void readyToAnimate();
    void setWindow(QQuickWindow *window);
    static QObject *findAnimationContext(QQuickAbstractAnimation *);

    QPointer<QQuickAnimatorController> m_controller;
    QQuickAbstractAnimation *m_animation;
    QAbstractAnimationJob *m_job;
    int m_duration;

    enum InternalState {
        State_Starting, // Used when it should be running, but no we're still missing the controller.
        State_Running,
        State_Paused,
        State_Stopped
    };

    InternalState m_internalState;
    bool m_jobManagedByController;
};

class Q_QUICK_PRIVATE_EXPORT QQuickAnimatorJob : public QAbstractAnimationJob
{
public:
    virtual void setTarget(QQuickItem *target);
    QQuickItem *target() const { return m_target; }

    void setFrom(qreal scale) { m_from = scale; }
    qreal from() const { return m_from; }

    void setTo(qreal to) { m_to = to; }
    qreal to() const { return m_to; }

    void setDuration(int duration) { m_duration = duration; }
    int duration() const Q_DECL_OVERRIDE { return m_duration; }

    QEasingCurve easingCurve() const { return m_easing; }
    void setEasingCurve(const QEasingCurve &curve) { m_easing = curve; }

    virtual void targetWasDeleted();
    virtual void initialize(QQuickAnimatorController *controller);
    virtual void writeBack() = 0;
    virtual void nodeWasDestroyed() = 0;

    bool isTransform() const { return m_isTransform; }
    bool isUniform() const { return m_isUniform; }

    bool hasBeenRunning() const { return m_hasBeenRunning; }
    void setHasBeenRunning(bool has) { m_hasBeenRunning = has; }

    qreal value() const;

    QQuickAnimatorController *controller() const { return m_controller; }

protected:
    QQuickAnimatorJob();
    void debugAnimation(QDebug d) const Q_DECL_OVERRIDE;

    QQuickItem *m_target;
    QQuickAnimatorController *m_controller;

    qreal m_from;
    qreal m_to;
    qreal m_value;

    QEasingCurve m_easing;

    int m_duration;

    uint m_feedback : 1;
    uint m_isTransform : 1;
    uint m_isUniform : 1;
    uint m_hasBeenRunning : 1;
};

class QQuickTransformAnimatorJob : public QQuickAnimatorJob
{
public:

    struct Helper
    {
        Helper()
            : ref(1)
            , node(0)
            , ox(0)
            , oy(0)
            , dx(0)
            , dy(0)
            , scale(1)
            , rotation(0)
            , wasSynced(false)
            , wasChanged(false)
        {
        }

        void sync();
        void apply();

        int ref;
        QQuickItem *item;
        QSGTransformNode *node;

        // Origin
        float ox;
        float oy;

        float dx;
        float dy;
        float scale;
        float rotation;

        uint wasSynced : 1;
        uint wasChanged : 1;
    };

    ~QQuickTransformAnimatorJob();
    Helper *transformHelper() const { return m_helper; }

protected:
    QQuickTransformAnimatorJob();
    void initialize(QQuickAnimatorController *controller) Q_DECL_OVERRIDE;
    void nodeWasDestroyed() Q_DECL_OVERRIDE;
    void targetWasDeleted() Q_DECL_OVERRIDE;

    Helper *m_helper;
};

class Q_QUICK_PRIVATE_EXPORT QQuickScaleAnimatorJob : public QQuickTransformAnimatorJob
{
public:
    void updateCurrentTime(int time);
    void writeBack();
};

class Q_QUICK_PRIVATE_EXPORT QQuickXAnimatorJob : public QQuickTransformAnimatorJob
{
public:
    void updateCurrentTime(int time);
    void writeBack();
};

class Q_QUICK_PRIVATE_EXPORT QQuickYAnimatorJob : public QQuickTransformAnimatorJob
{
public:
    void updateCurrentTime(int time);
    void writeBack();
};

class Q_QUICK_PRIVATE_EXPORT QQuickRotationAnimatorJob : public QQuickTransformAnimatorJob
{
public:
    QQuickRotationAnimatorJob();

    void updateCurrentTime(int time);
    void writeBack();

    void setDirection(QQuickRotationAnimator::RotationDirection direction) { m_direction = direction; }
    QQuickRotationAnimator::RotationDirection direction() const { return m_direction; }

private:
    QQuickRotationAnimator::RotationDirection m_direction;
};

class Q_QUICK_PRIVATE_EXPORT QQuickOpacityAnimatorJob : public QQuickAnimatorJob
{
public:
    QQuickOpacityAnimatorJob();

    void initialize(QQuickAnimatorController *controller);
    void updateCurrentTime(int time);
    void writeBack();
    void nodeWasDestroyed();

private:
    QSGOpacityNode *m_opacityNode;
};

class Q_QUICK_PRIVATE_EXPORT QQuickUniformAnimatorJob : public QQuickAnimatorJob
{
public:
    QQuickUniformAnimatorJob();

    void setTarget(QQuickItem *target);

    void setUniform(const QByteArray &uniform) { m_uniform = uniform; }
    QByteArray uniform() const { return m_uniform; }

    void afterNodeSync();

    void updateCurrentTime(int time);
    void writeBack();
    void nodeWasDestroyed();

private:
    QByteArray m_uniform;
    QQuickShaderEffectNode *m_node;

    int m_uniformIndex : 8;
    int m_uniformType : 8;
};

QT_END_NAMESPACE

#endif // QQUICKANIMATORJOB_P_H
