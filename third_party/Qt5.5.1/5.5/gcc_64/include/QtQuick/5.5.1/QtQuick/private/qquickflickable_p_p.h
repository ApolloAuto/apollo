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

#ifndef QQUICKFLICKABLE_P_P_H
#define QQUICKFLICKABLE_P_P_H

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

#include "qquickflickable_p.h"
#include "qquickitem_p.h"
#include "qquickitemchangelistener_p.h"

#include <QtQml/qqml.h>
#include <QtCore/qdatetime.h>
#include "qplatformdefs.h"

#include <private/qquicktimeline_p_p.h>
#include <private/qquickanimation_p_p.h>
#include <private/qquicktransitionmanager_p_p.h>

QT_BEGIN_NAMESPACE

// Really slow flicks can be annoying.
const qreal MinimumFlickVelocity = 75.0;

class QQuickFlickableVisibleArea;
class QQuickTransition;
class QQuickFlickableReboundTransition;

class Q_AUTOTEST_EXPORT QQuickFlickablePrivate : public QQuickItemPrivate, public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickFlickable)

public:
    static inline QQuickFlickablePrivate *get(QQuickFlickable *o) { return o->d_func(); }

    QQuickFlickablePrivate();
    void init();

    struct Velocity : public QQuickTimeLineValue
    {
        Velocity(QQuickFlickablePrivate *p)
            : parent(p) {}
        void setValue(qreal v) Q_DECL_OVERRIDE {
            if (v != value()) {
                QQuickTimeLineValue::setValue(v);
                parent->updateVelocity();
            }
        }
        QQuickFlickablePrivate *parent;
    };

    struct AxisData {
        AxisData(QQuickFlickablePrivate *fp, void (QQuickFlickablePrivate::*func)(qreal))
            : move(fp, func)
            , transitionToBounds(0)
            , viewSize(-1), lastPos(0), previousDragDelta(0), velocity(0), startMargin(0), endMargin(0)
            , origin(0)
            , transitionTo(0)
            , continuousFlickVelocity(0), velocityTime(), vTime(0)
            , smoothVelocity(fp), atEnd(false), atBeginning(true)
            , transitionToSet(false)
            , fixingUp(false), inOvershoot(false), moving(false), flicking(false)
            , dragging(false), extentsChanged(false)
            , explicitValue(false), minExtentDirty(true), maxExtentDirty(true)
        {}

        ~AxisData();

        void reset() {
            velocityBuffer.clear();
            dragStartOffset = 0;
            fixingUp = false;
            inOvershoot = false;
        }

        void markExtentsDirty() {
            minExtentDirty = true;
            maxExtentDirty = true;
            extentsChanged = true;
        }

        void resetTransitionTo() {
            transitionTo = 0;
            transitionToSet = false;
        }

        void addVelocitySample(qreal v, qreal maxVelocity);
        void updateVelocity();

        QQuickTimeLineValueProxy<QQuickFlickablePrivate> move;
        QQuickFlickableReboundTransition *transitionToBounds;
        qreal viewSize;
        qreal pressPos;
        qreal lastPos;
        qreal dragStartOffset;
        qreal dragMinBound;
        qreal dragMaxBound;
        qreal previousDragDelta;
        qreal velocity;
        qreal flickTarget;
        qreal startMargin;
        qreal endMargin;
        qreal origin;
        qreal transitionTo;
        qreal continuousFlickVelocity;
        QElapsedTimer velocityTime;
        int vTime;
        QQuickFlickablePrivate::Velocity smoothVelocity;
        QPODVector<qreal,10> velocityBuffer;
        bool atEnd : 1;
        bool atBeginning : 1;
        bool transitionToSet : 1;
        bool fixingUp : 1;
        bool inOvershoot : 1;
        bool inRebound : 1;
        bool moving : 1;
        bool flicking : 1;
        bool dragging : 1;
        bool extentsChanged : 1;
        bool explicitValue : 1;
        mutable bool minExtentDirty : 1;
        mutable bool maxExtentDirty : 1;
    };

    bool flickX(qreal velocity);
    bool flickY(qreal velocity);
    virtual bool flick(AxisData &data, qreal minExtent, qreal maxExtent, qreal vSize,
                        QQuickTimeLineCallback::Callback fixupCallback, qreal velocity);
    void flickingStarted(bool flickingH, bool flickingV);

    void fixupX();
    void fixupY();
    virtual void fixup(AxisData &data, qreal minExtent, qreal maxExtent);
    void adjustContentPos(AxisData &data, qreal toPos);
    void resetTimeline(AxisData &data);
    void clearTimeline();

    void updateBeginningEnd();

    bool isInnermostPressDelay(QQuickItem *item) const;
    void captureDelayedPress(QQuickItem *item, QMouseEvent *event);
    void clearDelayedPress();
    void replayDelayedPress();

    void setViewportX(qreal x);
    void setViewportY(qreal y);

    qreal overShootDistance(qreal size);

    void itemGeometryChanged(QQuickItem *, const QRectF &, const QRectF &) Q_DECL_OVERRIDE;

    void draggingStarting();
    void draggingEnding();

    bool isViewMoving() const;

    void cancelInteraction();

public:
    QQuickItem *contentItem;

    AxisData hData;
    AxisData vData;

    QQuickTimeLine timeline;
    bool hMoved : 1;
    bool vMoved : 1;
    bool stealMouse : 1;
    bool pressed : 1;
    bool scrollingPhase : 1;
    bool interactive : 1;
    bool calcVelocity : 1;
    bool pixelAligned : 1;
    bool replayingPressEvent : 1;
    QElapsedTimer timer;
    qint64 lastPosTime;
    qint64 lastPressTime;
    QPointF lastPos;
    QPointF pressPos;
    QVector2D accumulatedWheelPixelDelta;
    qreal deceleration;
    qreal maxVelocity;
    qreal reportedVelocitySmoothing;
    QMouseEvent *delayedPressEvent;
    QBasicTimer delayedPressTimer;
    int pressDelay;
    int fixupDuration;
    qreal flickBoost;

    enum FixupMode { Normal, Immediate, ExtentChanged };
    FixupMode fixupMode;

    static void fixupY_callback(void *);
    static void fixupX_callback(void *);

    void updateVelocity();
    int vTime;
    QQuickTimeLine velocityTimeline;
    QQuickFlickableVisibleArea *visibleArea;
    QQuickFlickable::FlickableDirection flickableDirection;
    QQuickFlickable::BoundsBehavior boundsBehavior;
    QQuickTransition *rebound;

    void viewportAxisMoved(AxisData &data, qreal minExtent, qreal maxExtent, qreal vSize,
                       QQuickTimeLineCallback::Callback fixupCallback);

    void handleMousePressEvent(QMouseEvent *);
    void handleMouseMoveEvent(QMouseEvent *);
    void handleMouseReleaseEvent(QMouseEvent *);

    void maybeBeginDrag(qint64 currentTimestamp, const QPointF &pressPosn);
    void drag(qint64 currentTimestamp, QEvent::Type eventType, const QPointF &localPos,
              const QVector2D &deltas, bool overThreshold, bool momentum,
              bool velocitySensitiveOverBounds, const QVector2D &velocity);

    qint64 computeCurrentTime(QInputEvent *event);
    qreal devicePixelRatio();

    // flickableData property
    static void data_append(QQmlListProperty<QObject> *, QObject *);
    static int data_count(QQmlListProperty<QObject> *);
    static QObject *data_at(QQmlListProperty<QObject> *, int);
    static void data_clear(QQmlListProperty<QObject> *);
};

class QQuickFlickableVisibleArea : public QObject
{
    Q_OBJECT

    Q_PROPERTY(qreal xPosition READ xPosition NOTIFY xPositionChanged)
    Q_PROPERTY(qreal yPosition READ yPosition NOTIFY yPositionChanged)
    Q_PROPERTY(qreal widthRatio READ widthRatio NOTIFY widthRatioChanged)
    Q_PROPERTY(qreal heightRatio READ heightRatio NOTIFY heightRatioChanged)

public:
    QQuickFlickableVisibleArea(QQuickFlickable *parent=0);

    qreal xPosition() const;
    qreal widthRatio() const;
    qreal yPosition() const;
    qreal heightRatio() const;

    void updateVisible();

Q_SIGNALS:
    void xPositionChanged(qreal xPosition);
    void yPositionChanged(qreal yPosition);
    void widthRatioChanged(qreal widthRatio);
    void heightRatioChanged(qreal heightRatio);

private:
    QQuickFlickable *flickable;
    qreal m_xPosition;
    qreal m_widthRatio;
    qreal m_yPosition;
    qreal m_heightRatio;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickFlickableVisibleArea)

#endif // QQUICKFLICKABLE_P_P_H
