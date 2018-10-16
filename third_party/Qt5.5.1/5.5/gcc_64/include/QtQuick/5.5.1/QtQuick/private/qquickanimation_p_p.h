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

#ifndef QQUICKANIMATION2_P_H
#define QQUICKANIMATION2_P_H

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

#include "qquickanimation_p.h"

#include <private/qqmlnullablevalue_p.h>

#include <qqml.h>
#include <qqmlcontext.h>

#include <private/qvariantanimation_p.h>
#include "private/qpauseanimationjob_p.h"
#include <QDebug>

#include <private/qobject_p.h>
#include "private/qanimationgroupjob_p.h"
#include <QDebug>

#include <private/qobject_p.h>


QT_BEGIN_NAMESPACE

//interface for classes that provide animation actions for QActionAnimation
class QAbstractAnimationAction
{
public:
    virtual ~QAbstractAnimationAction() {}
    virtual void doAction() = 0;
    virtual void debugAction(QDebug, int) const {}
};

//templated animation action
//allows us to specify an action that calls a function of a class.
//(so that class doesn't have to inherit QQuickAbstractAnimationAction)
template<class T, void (T::*method)(), void (T::*debugMethod)(QDebug, int) const>
class QAnimationActionProxy : public QAbstractAnimationAction
{
public:
    QAnimationActionProxy(T *instance) : m_instance(instance) {}
    virtual void doAction() { (m_instance->*method)(); }
    virtual void debugAction(QDebug d, int indentLevel) const { (m_instance->*debugMethod)(d, indentLevel); }
private:
    T *m_instance;
};

//performs an action of type QAbstractAnimationAction
class Q_AUTOTEST_EXPORT QActionAnimation : public QAbstractAnimationJob
{
    Q_DISABLE_COPY(QActionAnimation)
public:
    QActionAnimation();

    QActionAnimation(QAbstractAnimationAction *action);
    ~QActionAnimation();

    virtual int duration() const;
    void setAnimAction(QAbstractAnimationAction *action);

protected:
    virtual void updateCurrentTime(int);
    virtual void updateState(State newState, State oldState);
    void debugAnimation(QDebug d) const;

private:
    QAbstractAnimationAction *animAction;
};

class QQuickBulkValueUpdater
{
public:
    virtual ~QQuickBulkValueUpdater() {}
    virtual void setValue(qreal value) = 0;
    virtual void debugUpdater(QDebug, int) const {}
};

//animates QQuickBulkValueUpdater (assumes start and end values will be reals or compatible)
class Q_AUTOTEST_EXPORT QQuickBulkValueAnimator : public QAbstractAnimationJob
{
    Q_DISABLE_COPY(QQuickBulkValueAnimator)
public:
    QQuickBulkValueAnimator();
    ~QQuickBulkValueAnimator();

    void setAnimValue(QQuickBulkValueUpdater *value);
    QQuickBulkValueUpdater *getAnimValue() const { return animValue; }

    void setFromSourcedValue(bool *value) { fromSourced = value; }

    int duration() const { return m_duration; }
    void setDuration(int msecs) { m_duration = msecs; }

    QEasingCurve easingCurve() const { return easing; }
    void setEasingCurve(const QEasingCurve &curve) { easing = curve; }

protected:
    void updateCurrentTime(int currentTime);
    void topLevelAnimationLoopChanged();
    void debugAnimation(QDebug d) const;

private:
    QQuickBulkValueUpdater *animValue;
    bool *fromSourced;
    int m_duration;
    QEasingCurve easing;
};

//an animation that just gives a tick
template<class T, void (T::*method)(int)>
class QTickAnimationProxy : public QAbstractAnimationJob
{
    Q_DISABLE_COPY(QTickAnimationProxy)
public:
    QTickAnimationProxy(T *instance) : QAbstractAnimationJob(), m_instance(instance) {}
    virtual int duration() const { return -1; }
protected:
    virtual void updateCurrentTime(int msec) { (m_instance->*method)(msec); }

private:
    T *m_instance;
};

class QQuickAbstractAnimationPrivate : public QObjectPrivate, public QAnimationJobChangeListener
{
    Q_DECLARE_PUBLIC(QQuickAbstractAnimation)
public:
    QQuickAbstractAnimationPrivate()
    : running(false), paused(false), alwaysRunToEnd(false),
      /*connectedTimeLine(false), */componentComplete(true),
      avoidPropertyValueSourceStart(false), disableUserControl(false),
      registered(false), loopCount(1), group(0), animationInstance(0) {}

    bool running:1;
    bool paused:1;
    bool alwaysRunToEnd:1;
    //bool connectedTimeLine:1;
    bool componentComplete:1;
    bool avoidPropertyValueSourceStart:1;
    bool disableUserControl:1;
    bool registered:1;

    int loopCount;

    void commence();
    virtual void animationFinished(QAbstractAnimationJob *);

    QQmlProperty defaultProperty;

    QQuickAnimationGroup *group;
    QAbstractAnimationJob* animationInstance;

    static QQmlProperty createProperty(QObject *obj, const QString &str, QObject *infoObj);
};

class QQuickPauseAnimationPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickPauseAnimation)
public:
    QQuickPauseAnimationPrivate()
        : QQuickAbstractAnimationPrivate(), duration(250) {}

    int duration;
};

class QQuickScriptActionPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickScriptAction)
public:
    QQuickScriptActionPrivate();

    QQmlScriptString script;
    QString name;
    QQmlScriptString runScriptScript;
    bool hasRunScriptScript;
    bool reversing;

    void execute();
    QAbstractAnimationAction* createAction();
    void debugAction(QDebug d, int indentLevel) const;
    typedef QAnimationActionProxy<QQuickScriptActionPrivate,
                                 &QQuickScriptActionPrivate::execute,
                                 &QQuickScriptActionPrivate::debugAction> Proxy;
};

class QQuickPropertyActionPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickPropertyAction)
public:
    QQuickPropertyActionPrivate()
    : QQuickAbstractAnimationPrivate(), target(0) {}

    QObject *target;
    QString propertyName;
    QString properties;
    QList<QObject *> targets;
    QList<QObject *> exclude;

    QQmlNullableValue<QVariant> value;
};

class QQuickAnimationGroupPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickAnimationGroup)
public:
    QQuickAnimationGroupPrivate()
    : QQuickAbstractAnimationPrivate() {}

    static void append_animation(QQmlListProperty<QQuickAbstractAnimation> *list, QQuickAbstractAnimation *role);
    static void clear_animation(QQmlListProperty<QQuickAbstractAnimation> *list);
    QList<QQuickAbstractAnimation *> animations;
};

class QQuickPropertyAnimationPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickPropertyAnimation)
public:
    QQuickPropertyAnimationPrivate()
    : QQuickAbstractAnimationPrivate(), target(0), fromSourced(false), fromIsDefined(false), toIsDefined(false),
      defaultToInterpolatorType(0), interpolatorType(0), interpolator(0), duration(250), actions(0) {}

    QVariant from;
    QVariant to;

    QObject *target;
    QString propertyName;
    QString properties;
    QList<QObject *> targets;
    QList<QObject *> exclude;
    QString defaultProperties;

    bool fromSourced;
    bool fromIsDefined:1;
    bool toIsDefined:1;
    bool defaultToInterpolatorType:1;
    int interpolatorType;
    QVariantAnimation::Interpolator interpolator;
    int duration;
    QEasingCurve easing;

    // for animations that don't use the QQuickBulkValueAnimator
    QQuickStateActions *actions;

    static QVariant interpolateVariant(const QVariant &from, const QVariant &to, qreal progress);
    static void convertVariant(QVariant &variant, int type);
};

class QQuickRotationAnimationPrivate : public QQuickPropertyAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickRotationAnimation)
public:
    QQuickRotationAnimationPrivate() : direction(QQuickRotationAnimation::Numerical) {}

    QQuickRotationAnimation::RotationDirection direction;
};

class Q_AUTOTEST_EXPORT QQuickAnimationPropertyUpdater : public QQuickBulkValueUpdater
{
public:
    QQuickAnimationPropertyUpdater() : interpolatorType(0), interpolator(0), prevInterpolatorType(0), reverse(false), fromSourced(false), fromDefined(false), wasDeleted(0) {}
    ~QQuickAnimationPropertyUpdater();

    void setValue(qreal v);

    void debugUpdater(QDebug d, int indentLevel) const;

    QQuickStateActions actions;
    int interpolatorType;       //for Number/ColorAnimation
    QVariantAnimation::Interpolator interpolator;
    int prevInterpolatorType;   //for generic
    bool reverse;
    bool fromSourced;
    bool fromDefined;
    bool *wasDeleted;
};

QT_END_NAMESPACE

#endif // QQUICKANIMATION2_P_H
