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

#ifndef QQUICKANIMATION_H
#define QQUICKANIMATION_H

#include "qquickstate_p.h"
#include <QtGui/qvector3d.h>

#include <qqmlpropertyvaluesource.h>
#include <qqml.h>
#include <qqmlscriptstring.h>

#include <QtCore/qvariant.h>
#include <QtCore/qeasingcurve.h>
#include "private/qabstractanimationjob_p.h"
#include <QtGui/qcolor.h>

QT_BEGIN_NAMESPACE

class QQuickAbstractAnimationPrivate;
class QQuickAnimationGroup;
class Q_QUICK_PRIVATE_EXPORT QQuickAbstractAnimation : public QObject, public QQmlPropertyValueSource, public QQmlParserStatus
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAbstractAnimation)

    Q_INTERFACES(QQmlParserStatus)
    Q_INTERFACES(QQmlPropertyValueSource)
    Q_ENUMS(Loops)
    Q_PROPERTY(bool running READ isRunning WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool paused READ isPaused WRITE setPaused NOTIFY pausedChanged)
    Q_PROPERTY(bool alwaysRunToEnd READ alwaysRunToEnd WRITE setAlwaysRunToEnd NOTIFY alwaysRunToEndChanged)
    Q_PROPERTY(int loops READ loops WRITE setLoops NOTIFY loopCountChanged)
    Q_CLASSINFO("DefaultMethod", "start()")

public:
    enum ThreadingModel  {
        GuiThread,
        RenderThread,
        AnyThread
    };

    QQuickAbstractAnimation(QObject *parent=0);
    virtual ~QQuickAbstractAnimation();

    enum Loops { Infinite = -2 };

    bool isRunning() const;
    void setRunning(bool);
    bool isPaused() const;
    void setPaused(bool);
    bool alwaysRunToEnd() const;
    void setAlwaysRunToEnd(bool);

    int loops() const;
    void setLoops(int);
    int duration() const;

    int currentTime();
    void setCurrentTime(int);

    QQuickAnimationGroup *group() const;
    void setGroup(QQuickAnimationGroup *);

    void setDefaultTarget(const QQmlProperty &);
    void setDisableUserControl();
    void setEnableUserControl();
    bool userControlDisabled() const;
    void classBegin();
    void componentComplete();

    virtual ThreadingModel threadingModel() const;

Q_SIGNALS:
    void started();
    void stopped();
    void runningChanged(bool);
    void pausedChanged(bool);
    void alwaysRunToEndChanged(bool);
    void loopCountChanged(int);

public Q_SLOTS:
    void restart();
    void start();
    void pause();
    void resume();
    void stop();
    void complete();

protected:
    QQuickAbstractAnimation(QQuickAbstractAnimationPrivate &dd, QObject *parent);
    QAbstractAnimationJob* initInstance(QAbstractAnimationJob *animation);

public:
    enum TransitionDirection { Forward, Backward };
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
    QAbstractAnimationJob* qtAnimation();

private Q_SLOTS:
    void componentFinalized();
private:
    virtual void setTarget(const QQmlProperty &);
    void notifyRunningChanged(bool running);
    friend class QQuickBehavior;
    friend class QQuickBehaviorPrivate;
    friend class QQuickAnimationGroup;
};

class QQuickPauseAnimationPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickPauseAnimation : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPauseAnimation)

    Q_PROPERTY(int duration READ duration WRITE setDuration NOTIFY durationChanged)

public:
    QQuickPauseAnimation(QObject *parent=0);
    virtual ~QQuickPauseAnimation();

    int duration() const;
    void setDuration(int);

Q_SIGNALS:
    void durationChanged(int);

protected:
    QAbstractAnimationJob* transition(QQuickStateActions &actions,
                                          QQmlProperties &modified,
                                          TransitionDirection direction,
                                          QObject *defaultTarget = 0);
};

class QQuickScriptActionPrivate;
class QQuickScriptAction : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickScriptAction)

    Q_PROPERTY(QQmlScriptString script READ script WRITE setScript)
    Q_PROPERTY(QString scriptName READ stateChangeScriptName WRITE setStateChangeScriptName)

public:
    QQuickScriptAction(QObject *parent=0);
    virtual ~QQuickScriptAction();

    QQmlScriptString script() const;
    void setScript(const QQmlScriptString &);

    QString stateChangeScriptName() const;
    void setStateChangeScriptName(const QString &);

protected:
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
};

class QQuickPropertyActionPrivate;
class QQuickPropertyAction : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPropertyAction)

    Q_PROPERTY(QObject *target READ target WRITE setTargetObject NOTIFY targetChanged)
    Q_PROPERTY(QString property READ property WRITE setProperty NOTIFY propertyChanged)
    Q_PROPERTY(QString properties READ properties WRITE setProperties NOTIFY propertiesChanged)
    Q_PROPERTY(QQmlListProperty<QObject> targets READ targets)
    Q_PROPERTY(QQmlListProperty<QObject> exclude READ exclude)
    Q_PROPERTY(QVariant value READ value WRITE setValue NOTIFY valueChanged)

public:
    QQuickPropertyAction(QObject *parent=0);
    virtual ~QQuickPropertyAction();

    QObject *target() const;
    void setTargetObject(QObject *);

    QString property() const;
    void setProperty(const QString &);

    QString properties() const;
    void setProperties(const QString &);

    QQmlListProperty<QObject> targets();
    QQmlListProperty<QObject> exclude();

    QVariant value() const;
    void setValue(const QVariant &);

Q_SIGNALS:
    void valueChanged(const QVariant &);
    void propertiesChanged(const QString &);
    void targetChanged();
    void propertyChanged();

protected:
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
};

class QQuickPropertyAnimationPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickPropertyAnimation : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPropertyAnimation)

    Q_PROPERTY(int duration READ duration WRITE setDuration NOTIFY durationChanged)
    Q_PROPERTY(QVariant from READ from WRITE setFrom NOTIFY fromChanged)
    Q_PROPERTY(QVariant to READ to WRITE setTo NOTIFY toChanged)
    Q_PROPERTY(QEasingCurve easing READ easing WRITE setEasing NOTIFY easingChanged)
    Q_PROPERTY(QObject *target READ target WRITE setTargetObject NOTIFY targetChanged)
    Q_PROPERTY(QString property READ property WRITE setProperty NOTIFY propertyChanged)
    Q_PROPERTY(QString properties READ properties WRITE setProperties NOTIFY propertiesChanged)
    Q_PROPERTY(QQmlListProperty<QObject> targets READ targets)
    Q_PROPERTY(QQmlListProperty<QObject> exclude READ exclude)

public:
    QQuickPropertyAnimation(QObject *parent=0);
    virtual ~QQuickPropertyAnimation();

    virtual int duration() const;
    virtual void setDuration(int);

    QVariant from() const;
    void setFrom(const QVariant &);

    QVariant to() const;
    void setTo(const QVariant &);

    QEasingCurve easing() const;
    void setEasing(const QEasingCurve &);

    QObject *target() const;
    void setTargetObject(QObject *);

    QString property() const;
    void setProperty(const QString &);

    QString properties() const;
    void setProperties(const QString &);

    QQmlListProperty<QObject> targets();
    QQmlListProperty<QObject> exclude();

protected:
    QQuickStateActions createTransitionActions(QQuickStateActions &actions,
                                                     QQmlProperties &modified,
                                                     QObject *defaultTarget = 0);

    QQuickPropertyAnimation(QQuickPropertyAnimationPrivate &dd, QObject *parent);
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
Q_SIGNALS:
    void durationChanged(int);
    void fromChanged(const QVariant &);
    void toChanged(const QVariant &);
    void easingChanged(const QEasingCurve &);
    void propertiesChanged(const QString &);
    void targetChanged();
    void propertyChanged();
};

class Q_QUICK_PRIVATE_EXPORT QQuickColorAnimation : public QQuickPropertyAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPropertyAnimation)
    Q_PROPERTY(QColor from READ from WRITE setFrom)
    Q_PROPERTY(QColor to READ to WRITE setTo)

public:
    QQuickColorAnimation(QObject *parent=0);
    virtual ~QQuickColorAnimation();

    QColor from() const;
    void setFrom(const QColor &);

    QColor to() const;
    void setTo(const QColor &);
};

class Q_QUICK_PRIVATE_EXPORT QQuickNumberAnimation : public QQuickPropertyAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPropertyAnimation)

    Q_PROPERTY(qreal from READ from WRITE setFrom)
    Q_PROPERTY(qreal to READ to WRITE setTo)

public:
    QQuickNumberAnimation(QObject *parent=0);
    virtual ~QQuickNumberAnimation();

    qreal from() const;
    void setFrom(qreal);

    qreal to() const;
    void setTo(qreal);

protected:
    QQuickNumberAnimation(QQuickPropertyAnimationPrivate &dd, QObject *parent);

private:
    void init();
};

class Q_QUICK_PRIVATE_EXPORT QQuickVector3dAnimation : public QQuickPropertyAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPropertyAnimation)

    Q_PROPERTY(QVector3D from READ from WRITE setFrom)
    Q_PROPERTY(QVector3D to READ to WRITE setTo)

public:
    QQuickVector3dAnimation(QObject *parent=0);
    virtual ~QQuickVector3dAnimation();

    QVector3D from() const;
    void setFrom(QVector3D);

    QVector3D to() const;
    void setTo(QVector3D);
};

class QQuickRotationAnimationPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickRotationAnimation : public QQuickPropertyAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickRotationAnimation)
    Q_ENUMS(RotationDirection)

    Q_PROPERTY(qreal from READ from WRITE setFrom)
    Q_PROPERTY(qreal to READ to WRITE setTo)
    Q_PROPERTY(RotationDirection direction READ direction WRITE setDirection NOTIFY directionChanged)

public:
    QQuickRotationAnimation(QObject *parent=0);
    virtual ~QQuickRotationAnimation();

    qreal from() const;
    void setFrom(qreal);

    qreal to() const;
    void setTo(qreal);

    enum RotationDirection { Numerical, Shortest, Clockwise, Counterclockwise };
    RotationDirection direction() const;
    void setDirection(RotationDirection direction);

Q_SIGNALS:
    void directionChanged();
};

class QQuickAnimationGroupPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickAnimationGroup : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAnimationGroup)

    Q_CLASSINFO("DefaultProperty", "animations")
    Q_PROPERTY(QQmlListProperty<QQuickAbstractAnimation> animations READ animations)

public:
    QQuickAnimationGroup(QObject *parent);
    virtual ~QQuickAnimationGroup();

    QQmlListProperty<QQuickAbstractAnimation> animations();
    friend class QQuickAbstractAnimation;

protected:
    QQuickAnimationGroup(QQuickAnimationGroupPrivate &dd, QObject *parent);
};

class QQuickSequentialAnimation : public QQuickAnimationGroup
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAnimationGroup)

public:
    QQuickSequentialAnimation(QObject *parent=0);
    virtual ~QQuickSequentialAnimation();

protected:
    virtual ThreadingModel threadingModel() const;
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
};

class Q_QUICK_PRIVATE_EXPORT QQuickParallelAnimation : public QQuickAnimationGroup
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAnimationGroup)

public:
    QQuickParallelAnimation(QObject *parent=0);
    virtual ~QQuickParallelAnimation();

protected:
    virtual ThreadingModel threadingModel() const;
    virtual QAbstractAnimationJob* transition(QQuickStateActions &actions,
                            QQmlProperties &modified,
                            TransitionDirection direction,
                            QObject *defaultTarget = 0);
};


QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickAbstractAnimation)
QML_DECLARE_TYPE(QQuickPauseAnimation)
QML_DECLARE_TYPE(QQuickScriptAction)
QML_DECLARE_TYPE(QQuickPropertyAction)
QML_DECLARE_TYPE(QQuickPropertyAnimation)
QML_DECLARE_TYPE(QQuickColorAnimation)
QML_DECLARE_TYPE(QQuickNumberAnimation)
QML_DECLARE_TYPE(QQuickSequentialAnimation)
QML_DECLARE_TYPE(QQuickParallelAnimation)
QML_DECLARE_TYPE(QQuickVector3dAnimation)
QML_DECLARE_TYPE(QQuickRotationAnimation)

#endif // QQUICKANIMATION_H
