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

#ifndef QQUICKFLICKABLE_P_H
#define QQUICKFLICKABLE_P_H

#include "qquickitem.h"
#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE

class QQuickFlickablePrivate;
class QQuickFlickableVisibleArea;
class Q_QUICK_PRIVATE_EXPORT QQuickFlickable : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(qreal contentWidth READ contentWidth WRITE setContentWidth NOTIFY contentWidthChanged)
    Q_PROPERTY(qreal contentHeight READ contentHeight WRITE setContentHeight NOTIFY contentHeightChanged)
    Q_PROPERTY(qreal contentX READ contentX WRITE setContentX NOTIFY contentXChanged)
    Q_PROPERTY(qreal contentY READ contentY WRITE setContentY NOTIFY contentYChanged)
    Q_PROPERTY(QQuickItem *contentItem READ contentItem CONSTANT)

    Q_PROPERTY(qreal topMargin READ topMargin WRITE setTopMargin NOTIFY topMarginChanged)
    Q_PROPERTY(qreal bottomMargin READ bottomMargin WRITE setBottomMargin NOTIFY bottomMarginChanged)
    Q_PROPERTY(qreal originY READ originY NOTIFY originYChanged)

    Q_PROPERTY(qreal leftMargin READ leftMargin WRITE setLeftMargin NOTIFY leftMarginChanged)
    Q_PROPERTY(qreal rightMargin READ rightMargin WRITE setRightMargin NOTIFY rightMarginChanged)
    Q_PROPERTY(qreal originX READ originX NOTIFY originXChanged)

    Q_PROPERTY(qreal horizontalVelocity READ horizontalVelocity NOTIFY horizontalVelocityChanged)
    Q_PROPERTY(qreal verticalVelocity READ verticalVelocity NOTIFY verticalVelocityChanged)

    Q_PROPERTY(BoundsBehavior boundsBehavior READ boundsBehavior WRITE setBoundsBehavior NOTIFY boundsBehaviorChanged)
    Q_PROPERTY(QQuickTransition *rebound READ rebound WRITE setRebound NOTIFY reboundChanged)
    Q_PROPERTY(qreal maximumFlickVelocity READ maximumFlickVelocity WRITE setMaximumFlickVelocity NOTIFY maximumFlickVelocityChanged)
    Q_PROPERTY(qreal flickDeceleration READ flickDeceleration WRITE setFlickDeceleration NOTIFY flickDecelerationChanged)
    Q_PROPERTY(bool moving READ isMoving NOTIFY movingChanged)
    Q_PROPERTY(bool movingHorizontally READ isMovingHorizontally NOTIFY movingHorizontallyChanged)
    Q_PROPERTY(bool movingVertically READ isMovingVertically NOTIFY movingVerticallyChanged)
    Q_PROPERTY(bool flicking READ isFlicking NOTIFY flickingChanged)
    Q_PROPERTY(bool flickingHorizontally READ isFlickingHorizontally NOTIFY flickingHorizontallyChanged)
    Q_PROPERTY(bool flickingVertically READ isFlickingVertically NOTIFY flickingVerticallyChanged)
    Q_PROPERTY(bool dragging READ isDragging NOTIFY draggingChanged)
    Q_PROPERTY(bool draggingHorizontally READ isDraggingHorizontally NOTIFY draggingHorizontallyChanged)
    Q_PROPERTY(bool draggingVertically READ isDraggingVertically NOTIFY draggingVerticallyChanged)
    Q_PROPERTY(FlickableDirection flickableDirection READ flickableDirection WRITE setFlickableDirection NOTIFY flickableDirectionChanged)

    Q_PROPERTY(bool interactive READ isInteractive WRITE setInteractive NOTIFY interactiveChanged)
    Q_PROPERTY(int pressDelay READ pressDelay WRITE setPressDelay NOTIFY pressDelayChanged)

    Q_PROPERTY(bool atXEnd READ isAtXEnd NOTIFY isAtBoundaryChanged)
    Q_PROPERTY(bool atYEnd READ isAtYEnd NOTIFY isAtBoundaryChanged)
    Q_PROPERTY(bool atXBeginning READ isAtXBeginning NOTIFY isAtBoundaryChanged)
    Q_PROPERTY(bool atYBeginning READ isAtYBeginning NOTIFY isAtBoundaryChanged)

    Q_PROPERTY(QQuickFlickableVisibleArea *visibleArea READ visibleArea CONSTANT)

    Q_PROPERTY(bool pixelAligned READ pixelAligned WRITE setPixelAligned NOTIFY pixelAlignedChanged)

    Q_PROPERTY(QQmlListProperty<QObject> flickableData READ flickableData)
    Q_PROPERTY(QQmlListProperty<QQuickItem> flickableChildren READ flickableChildren)
    Q_CLASSINFO("DefaultProperty", "flickableData")

    Q_ENUMS(FlickableDirection)
    Q_FLAGS(BoundsBehavior)

public:
    QQuickFlickable(QQuickItem *parent=0);
    ~QQuickFlickable();

    QQmlListProperty<QObject> flickableData();
    QQmlListProperty<QQuickItem> flickableChildren();

    enum BoundsBehaviorFlag {
        StopAtBounds = 0x0,
        DragOverBounds = 0x1,
        OvershootBounds = 0x2,
        DragAndOvershootBounds = DragOverBounds | OvershootBounds
    };
    Q_DECLARE_FLAGS(BoundsBehavior, BoundsBehaviorFlag)

    BoundsBehavior boundsBehavior() const;
    void setBoundsBehavior(BoundsBehavior);

    QQuickTransition *rebound() const;
    void setRebound(QQuickTransition *transition);

    qreal contentWidth() const;
    void setContentWidth(qreal);

    qreal contentHeight() const;
    void setContentHeight(qreal);

    qreal contentX() const;
    virtual void setContentX(qreal pos);

    qreal contentY() const;
    virtual void setContentY(qreal pos);

    qreal topMargin() const;
    void setTopMargin(qreal m);

    qreal bottomMargin() const;
    void setBottomMargin(qreal m);

    qreal leftMargin() const;
    void setLeftMargin(qreal m);

    qreal rightMargin() const;
    void setRightMargin(qreal m);

    virtual qreal originY() const;
    virtual qreal originX() const;

    bool isMoving() const;
    bool isMovingHorizontally() const;
    bool isMovingVertically() const;
    bool isFlicking() const;
    bool isFlickingHorizontally() const;
    bool isFlickingVertically() const;
    bool isDragging() const;
    bool isDraggingHorizontally() const;
    bool isDraggingVertically() const;

    int pressDelay() const;
    void setPressDelay(int delay);

    qreal maximumFlickVelocity() const;
    void setMaximumFlickVelocity(qreal);

    qreal flickDeceleration() const;
    void setFlickDeceleration(qreal);

    bool isInteractive() const;
    void setInteractive(bool);

    qreal horizontalVelocity() const;
    qreal verticalVelocity() const;

    bool isAtXEnd() const;
    bool isAtXBeginning() const;
    bool isAtYEnd() const;
    bool isAtYBeginning() const;

    QQuickItem *contentItem();

    enum FlickableDirection { AutoFlickDirection=0x00, HorizontalFlick=0x01, VerticalFlick=0x02, HorizontalAndVerticalFlick=0x03 };
    FlickableDirection flickableDirection() const;
    void setFlickableDirection(FlickableDirection);

    bool pixelAligned() const;
    void setPixelAligned(bool align);

    Q_INVOKABLE void resizeContent(qreal w, qreal h, QPointF center);
    Q_INVOKABLE void returnToBounds();
    Q_INVOKABLE void flick(qreal xVelocity, qreal yVelocity);
    Q_INVOKABLE void cancelFlick();

Q_SIGNALS:
    void contentWidthChanged();
    void contentHeightChanged();
    void contentXChanged();
    void contentYChanged();
    void topMarginChanged();
    void bottomMarginChanged();
    void leftMarginChanged();
    void rightMarginChanged();
    void originYChanged();
    void originXChanged();
    void movingChanged();
    void movingHorizontallyChanged();
    void movingVerticallyChanged();
    void flickingChanged();
    void flickingHorizontallyChanged();
    void flickingVerticallyChanged();
    void draggingChanged();
    void draggingHorizontallyChanged();
    void draggingVerticallyChanged();
    void horizontalVelocityChanged();
    void verticalVelocityChanged();
    void isAtBoundaryChanged();
    void flickableDirectionChanged();
    void interactiveChanged();
    void boundsBehaviorChanged();
    void reboundChanged();
    void maximumFlickVelocityChanged();
    void flickDecelerationChanged();
    void pressDelayChanged();
    void movementStarted();
    void movementEnded();
    void flickStarted();
    void flickEnded();
    void dragStarted();
    void dragEnded();
    void pixelAlignedChanged();

protected:
    bool childMouseEventFilter(QQuickItem *, QEvent *) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
#ifndef QT_NO_WHEELEVENT
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
#endif
    void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

    QQuickFlickableVisibleArea *visibleArea();

protected Q_SLOTS:
    void movementStarting();
    void movementEnding();
    void movementEnding(bool hMovementEnding, bool vMovementEnding);
    void timelineCompleted();

protected:
    virtual qreal minXExtent() const;
    virtual qreal minYExtent() const;
    virtual qreal maxXExtent() const;
    virtual qreal maxYExtent() const;
    qreal vWidth() const;
    qreal vHeight() const;
    void componentComplete() Q_DECL_OVERRIDE;
    virtual void viewportMoved(Qt::Orientations orient);
    void geometryChanged(const QRectF &newGeometry,
                         const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void mouseUngrabEvent() Q_DECL_OVERRIDE;
    bool sendMouseEvent(QQuickItem *item, QMouseEvent *event);

    bool xflick() const;
    bool yflick() const;

protected:
    QQuickFlickable(QQuickFlickablePrivate &dd, QQuickItem *parent);

private:
    Q_DISABLE_COPY(QQuickFlickable)
    Q_DECLARE_PRIVATE(QQuickFlickable)
    friend class QQuickFlickableVisibleArea;
    friend class QQuickFlickableReboundTransition;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickFlickable)

#endif // QQUICKFLICKABLE_P_H
