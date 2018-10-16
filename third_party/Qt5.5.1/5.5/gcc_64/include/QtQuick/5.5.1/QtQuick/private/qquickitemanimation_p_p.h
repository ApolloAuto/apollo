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

#ifndef QQUICKANIMATION_P_H
#define QQUICKANIMATION_P_H

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

#include "qquickitemanimation_p.h"

#include <private/qquickpath_p.h>
#include <private/qquickanimation_p_p.h>

QT_BEGIN_NAMESPACE

class QQuickParentAnimationPrivate : public QQuickAnimationGroupPrivate
{
    Q_DECLARE_PUBLIC(QQuickParentAnimation)
public:
 QQuickParentAnimationPrivate()
    : QQuickAnimationGroupPrivate(), target(0), newParent(0), via(0) {}

    QQuickItem *target;
    QQuickItem *newParent;
    QQuickItem *via;

    QPointF computeTransformOrigin(QQuickItem::TransformOrigin origin, qreal width, qreal height) const;
};

class QQuickAnchorAnimationPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickAnchorAnimation)
public:
    QQuickAnchorAnimationPrivate() : interpolator(QVariantAnimationPrivate::getInterpolator(QMetaType::QReal)), duration(250) {}

    QVariantAnimation::Interpolator interpolator;
    int duration;
    QEasingCurve easing;
    QList<QQuickItem*> targets;
};

class QQuickPathAnimationUpdater : public QQuickBulkValueUpdater
{
public:
    QQuickPathAnimationUpdater() : path(0), pathLength(0), target(0), reverse(false),
        fromSourced(false), fromDefined(false), toDefined(false),
        toX(0), toY(0), currentV(0), orientation(QQuickPathAnimation::Fixed),
        entryInterval(0), exitInterval(0) {}
    ~QQuickPathAnimationUpdater() {}

    void setValue(qreal v);

    QQuickPath *path;

    QPainterPath painterPath;
    QQuickCachedBezier prevBez;
    qreal pathLength;
    QList<QQuickPath::AttributePoint> attributePoints;

    QQuickItem *target;
    bool reverse;
    bool fromSourced;
    bool fromDefined;
    bool toDefined;
    qreal toX;
    qreal toY;
    qreal currentV;
    QQmlNullableValue<qreal> interruptStart;
    //TODO: bundle below into common struct
    QQuickPathAnimation::Orientation orientation;
    QPointF anchorPoint;
    qreal entryInterval;
    qreal exitInterval;
    QQmlNullableValue<qreal> endRotation;
    QQmlNullableValue<qreal> startRotation;
};

class QQuickPathAnimationPrivate;
class QQuickPathAnimationAnimator : public QQuickBulkValueAnimator
{
public:
    QQuickPathAnimationAnimator(QQuickPathAnimationPrivate * = 0);
    ~QQuickPathAnimationAnimator();

    void clearTemplate() { animationTemplate = 0; }

    QQuickPathAnimationUpdater *pathUpdater() { return static_cast<QQuickPathAnimationUpdater*>(getAnimValue()); }
private:
    QQuickPathAnimationPrivate *animationTemplate;
};

class QQuickPathAnimationPrivate : public QQuickAbstractAnimationPrivate
{
    Q_DECLARE_PUBLIC(QQuickPathAnimation)
public:
    QQuickPathAnimationPrivate() : path(0), target(0),
        orientation(QQuickPathAnimation::Fixed), entryDuration(0), exitDuration(0), duration(250) {}

    QQuickPath *path;
    QQuickItem *target;
    QQuickPathAnimation::Orientation orientation;
    QPointF anchorPoint;
    qreal entryDuration;
    qreal exitDuration;
    QQmlNullableValue<qreal> endRotation;
    int duration;
    QEasingCurve easingCurve;
    QHash<QQuickItem*, QQuickPathAnimationAnimator* > activeAnimations;
};


QT_END_NAMESPACE

#endif // QQUICKANIMATION_P_H
