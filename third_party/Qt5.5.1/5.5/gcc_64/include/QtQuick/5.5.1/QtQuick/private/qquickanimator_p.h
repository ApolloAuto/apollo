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

#ifndef QQUICKANIMATOR_P_H
#define QQUICKANIMATOR_P_H

#include "qquickanimation_p.h"

QT_BEGIN_NAMESPACE

class QQuickItem;

class QQuickAnimatorJob;
class QQuickAnimatorPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickAnimator : public QQuickAbstractAnimation
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAnimator)
    Q_PROPERTY(QQuickItem *target READ targetItem WRITE setTargetItem NOTIFY targetItemChanged)
    Q_PROPERTY(QEasingCurve easing READ easing WRITE setEasing NOTIFY easingChanged)
    Q_PROPERTY(int duration READ duration WRITE setDuration NOTIFY durationChanged)
    Q_PROPERTY(qreal to READ to WRITE setTo NOTIFY toChanged)
    Q_PROPERTY(qreal from READ from WRITE setFrom NOTIFY fromChanged)

public:
    QQuickItem *targetItem() const;
    void setTargetItem(QQuickItem *target);

    int duration() const;
    void setDuration(int duration);

    QEasingCurve easing() const;
    void setEasing(const QEasingCurve & easing);

    qreal to() const;
    void setTo(qreal to);

    qreal from() const;
    void setFrom(qreal from);

protected:
    ThreadingModel threadingModel() const { return RenderThread; }
    virtual QQuickAnimatorJob *createJob() const = 0;
    virtual QString propertyName() const = 0;
    QAbstractAnimationJob *transition(QQuickStateActions &actions,
                                      QQmlProperties &modified,
                                      TransitionDirection,
                                      QObject *);

    QQuickAnimator(QQuickAnimatorPrivate &dd, QObject *parent = 0);
    QQuickAnimator(QObject *parent = 0);

Q_SIGNALS:
    void targetItemChanged(QQuickItem *);
    void durationChanged(int duration);
    void easingChanged(const QEasingCurve &curve);
    void toChanged(qreal to);
    void fromChanged(qreal from);
};

class QQuickScaleAnimatorPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickScaleAnimator : public QQuickAnimator
{
    Q_OBJECT
public:
    QQuickScaleAnimator(QObject *parent = 0);
protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const { return QStringLiteral("scale"); }
};

class Q_QUICK_PRIVATE_EXPORT QQuickXAnimator : public QQuickAnimator
{
    Q_OBJECT
public:
    QQuickXAnimator(QObject *parent = 0);
protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const{ return QStringLiteral("x"); }
};

class Q_QUICK_PRIVATE_EXPORT QQuickYAnimator : public QQuickAnimator
{
    Q_OBJECT
public:
    QQuickYAnimator(QObject *parent = 0);
protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const { return QStringLiteral("y"); }
};

class Q_QUICK_PRIVATE_EXPORT QQuickOpacityAnimator : public QQuickAnimator
{
    Q_OBJECT
public:
    QQuickOpacityAnimator(QObject *parent = 0);
protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const { return QStringLiteral("opacity"); }
};

class QQuickRotationAnimatorPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickRotationAnimator : public QQuickAnimator
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickRotationAnimator)
    Q_PROPERTY(RotationDirection direction READ direction WRITE setDirection NOTIFY directionChanged)

    Q_ENUMS(RotationDirection)

public:
    enum RotationDirection { Numerical, Shortest, Clockwise, Counterclockwise };

    QQuickRotationAnimator(QObject *parent = 0);

    void setDirection(RotationDirection dir);
    RotationDirection direction() const;

Q_SIGNALS:
    void directionChanged(RotationDirection dir);

protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const { return QStringLiteral("rotation"); }
};

class QQuickUniformAnimatorPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickUniformAnimator : public QQuickAnimator
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickUniformAnimator)
    Q_PROPERTY(QString uniform READ uniform WRITE setUniform NOTIFY uniformChanged)

public:
    QQuickUniformAnimator(QObject *parent = 0);

    QString uniform() const;
    void setUniform(const QString &);

Q_SIGNALS:
    void uniformChanged(const QString &);

protected:
    QQuickAnimatorJob *createJob() const;
    QString propertyName() const;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickAnimator)
QML_DECLARE_TYPE(QQuickXAnimator)
QML_DECLARE_TYPE(QQuickYAnimator)
QML_DECLARE_TYPE(QQuickScaleAnimator)
QML_DECLARE_TYPE(QQuickRotationAnimator)
QML_DECLARE_TYPE(QQuickOpacityAnimator)
QML_DECLARE_TYPE(QQuickUniformAnimator)

#endif // QQUICKANIMATOR_P_H
