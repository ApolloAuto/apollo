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

#ifndef QQUICKPATHVIEW_P_P_H
#define QQUICKPATHVIEW_P_P_H

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

#include "qquickpathview_p.h"
#include "qquickitem_p.h"

#include <QtQml/qqml.h>
#include <QtCore/qdatetime.h>
#include <QtCore/qcoreapplication.h>

#include <private/qquickanimation_p_p.h>
#include <private/qqmldelegatemodel_p.h>
#include <private/qquicktimeline_p_p.h>

QT_BEGIN_NAMESPACE

class QQmlOpenMetaObjectType;
class QQuickPathViewAttached;
class QQuickPathViewPrivate : public QQuickItemPrivate, public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickPathView)

public:
    QQuickPathViewPrivate();

    void init();

    void itemGeometryChanged(QQuickItem *item, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE {
        if ((newGeometry.size() != oldGeometry.size())
            && (!highlightItem || item != highlightItem)) {
            if (QQuickPathViewAttached *att = attached(item))
                att->m_percent = -1;
            scheduleLayout();
        }
    }

    void scheduleLayout() {
        Q_Q(QQuickPathView);
        if (!layoutScheduled) {
            layoutScheduled = true;
            q->polish();
        }
    }

    QQuickItem *getItem(int modelIndex, qreal z = 0, bool async=false);
    void releaseItem(QQuickItem *item);
    QQuickPathViewAttached *attached(QQuickItem *item);
    QQmlOpenMetaObjectType *attachedType();
    void clear();
    void updateMappedRange();
    qreal positionOfIndex(qreal index) const;
    bool isInBound(qreal position, qreal lower, qreal upper) const;
    void createHighlight();
    void updateHighlight();
    void setHighlightPosition(qreal pos);
    bool isValid() const {
        return model && model->count() > 0 && model->isValid() && path;
    }

    void handleMousePressEvent(QMouseEvent *event);
    void handleMouseMoveEvent(QMouseEvent *event);
    void handleMouseReleaseEvent(QMouseEvent *);

    int calcCurrentIndex();
    void createCurrentItem();
    void updateCurrent();
    static void fixOffsetCallback(void*);
    void fixOffset();
    void setOffset(qreal offset);
    void setAdjustedOffset(qreal offset);
    void regenerate();
    void updateItem(QQuickItem *, qreal);
    void snapToIndex(int index);
    QPointF pointNear(const QPointF &point, qreal *nearPercent=0) const;
    void addVelocitySample(qreal v);
    qreal calcVelocity() const;
    qint64 computeCurrentTime(QInputEvent *event);
    void setDragging(bool d);

    QQuickPath *path;
    int currentIndex;
    QPointer<QQuickItem> currentItem;
    qreal currentItemOffset;
    qreal startPc;
    QPointF startPoint;
    QPointF startPos;
    qreal offset;
    qreal offsetAdj;
    qreal mappedRange;
    qreal mappedCache;
    bool stealMouse : 1;
    bool ownModel : 1;
    bool interactive : 1;
    bool haveHighlightRange : 1;
    bool autoHighlight : 1;
    bool highlightUp : 1;
    bool layoutScheduled : 1;
    bool moving : 1;
    bool flicking : 1;
    bool dragging : 1;
    bool requestedOnPath : 1;
    bool inRequest : 1;
    bool delegateValidated : 1;
    bool inRefill : 1;
    QElapsedTimer timer;
    qint64 lastPosTime;
    QPointF lastPos;
    qreal dragMargin;
    qreal deceleration;
    qreal maximumFlickVelocity;
    QQuickTimeLine tl;
    QQuickTimeLineValueProxy<QQuickPathViewPrivate> moveOffset;
    int flickDuration;
    int firstIndex;
    int pathItems;
    int requestedIndex;
    int cacheSize;
    qreal requestedZ;
    QList<QQuickItem *> items;
    QList<QQuickItem *> itemCache;
    QPointer<QQmlInstanceModel> model;
    QVariant modelVariant;
    enum MovementReason { Other, SetIndex, Mouse };
    MovementReason moveReason;
    enum MovementDirection { Shortest, Negative, Positive };
    MovementDirection moveDirection;
    QQmlOpenMetaObjectType *attType;
    QQmlComponent *highlightComponent;
    QQuickItem *highlightItem;
    QQuickTimeLineValueProxy<QQuickPathViewPrivate> moveHighlight;
    qreal highlightPosition;
    qreal highlightRangeStart;
    qreal highlightRangeEnd;
    QQuickPathView::HighlightRangeMode highlightRangeMode;
    int highlightMoveDuration;
    int modelCount;
    QPODVector<qreal,10> velocityBuffer;
    QQuickPathView::SnapMode snapMode;
};

QT_END_NAMESPACE

#endif
