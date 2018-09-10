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

#ifndef QQUICKMOUSEAREA_P_H
#define QQUICKMOUSEAREA_P_H

#include "qquickitem.h"
#include <private/qtquickglobal_p.h>
#include <QtCore/qstringlist.h>

QT_BEGIN_NAMESPACE

class QQuickMouseEvent;
class QQuickDrag;
class QQuickMouseAreaPrivate;
class QQuickWheelEvent;
// used in Qt Location
class Q_QUICK_PRIVATE_EXPORT QQuickMouseArea : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(qreal mouseX READ mouseX NOTIFY mouseXChanged)
    Q_PROPERTY(qreal mouseY READ mouseY NOTIFY mouseYChanged)
    Q_PROPERTY(bool containsMouse READ hovered NOTIFY hoveredChanged)
    Q_PROPERTY(bool pressed READ pressed NOTIFY pressedChanged)
    Q_PROPERTY(bool enabled READ isEnabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(bool scrollGestureEnabled READ isScrollGestureEnabled WRITE setScrollGestureEnabled NOTIFY scrollGestureEnabledChanged REVISION 2)
    Q_PROPERTY(Qt::MouseButtons pressedButtons READ pressedButtons NOTIFY pressedButtonsChanged)
    Q_PROPERTY(Qt::MouseButtons acceptedButtons READ acceptedButtons WRITE setAcceptedButtons NOTIFY acceptedButtonsChanged)
    Q_PROPERTY(bool hoverEnabled READ hoverEnabled WRITE setHoverEnabled NOTIFY hoverEnabledChanged)
#ifndef QT_NO_DRAGANDDROP
    Q_PROPERTY(QQuickDrag *drag READ drag CONSTANT) //### add flicking to QQuickDrag or add a QQuickFlick ???
#endif
    Q_PROPERTY(bool preventStealing READ preventStealing WRITE setPreventStealing NOTIFY preventStealingChanged)
    Q_PROPERTY(bool propagateComposedEvents READ propagateComposedEvents WRITE setPropagateComposedEvents NOTIFY propagateComposedEventsChanged)
#ifndef QT_NO_CURSOR
    Q_PROPERTY(Qt::CursorShape cursorShape READ cursorShape WRITE setCursorShape RESET unsetCursor NOTIFY cursorShapeChanged)
#endif
    Q_PROPERTY(bool containsPress READ containsPress NOTIFY containsPressChanged REVISION 1)

public:
    QQuickMouseArea(QQuickItem *parent=0);
    ~QQuickMouseArea();

    qreal mouseX() const;
    qreal mouseY() const;

    bool isEnabled() const;
    void setEnabled(bool);

    bool isScrollGestureEnabled() const;
    void setScrollGestureEnabled(bool);

    bool hovered() const;
    bool pressed() const;
    bool containsPress() const;

    Qt::MouseButtons pressedButtons() const;

    Qt::MouseButtons acceptedButtons() const;
    void setAcceptedButtons(Qt::MouseButtons buttons);

    bool hoverEnabled() const;
    void setHoverEnabled(bool h);

#ifndef QT_NO_DRAGANDDROP
    QQuickDrag *drag();
#endif

    bool preventStealing() const;
    void setPreventStealing(bool prevent);

    bool propagateComposedEvents() const;
    void setPropagateComposedEvents(bool propagate);

#ifndef QT_NO_CURSOR
    Qt::CursorShape cursorShape() const;
    void setCursorShape(Qt::CursorShape shape);
#endif

Q_SIGNALS:
    void hoveredChanged();
    void pressedChanged();
    void enabledChanged();
    Q_REVISION(2) void scrollGestureEnabledChanged();
    void pressedButtonsChanged();
    void acceptedButtonsChanged();
    void hoverEnabledChanged();
#ifndef QT_NO_CURSOR
    void cursorShapeChanged();
#endif
    void positionChanged(QQuickMouseEvent *mouse);
    void mouseXChanged(QQuickMouseEvent *mouse);
    void mouseYChanged(QQuickMouseEvent *mouse);
    void preventStealingChanged();
    void propagateComposedEventsChanged();

    void pressed(QQuickMouseEvent *mouse);
    void pressAndHold(QQuickMouseEvent *mouse);
    void released(QQuickMouseEvent *mouse);
    void clicked(QQuickMouseEvent *mouse);
    void doubleClicked(QQuickMouseEvent *mouse);
    void wheel(QQuickWheelEvent *wheel);
    void entered();
    void exited();
    void canceled();
    Q_REVISION(1) void containsPressChanged();

protected:
    void setHovered(bool);
    bool setPressed(Qt::MouseButton button, bool);
    bool sendMouseEvent(QMouseEvent *event);

    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseUngrabEvent() Q_DECL_OVERRIDE;
    void hoverEnterEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void hoverMoveEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void hoverLeaveEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
#ifndef QT_NO_WHEELEVENT
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
#endif
    bool childMouseEventFilter(QQuickItem *i, QEvent *e) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
    void windowDeactivateEvent() Q_DECL_OVERRIDE;

    void geometryChanged(const QRectF &newGeometry,
                                 const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void itemChange(ItemChange change, const ItemChangeData& value) Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;

private:
    void handlePress();
    void handleRelease();
    void ungrabMouse();

private:
    Q_DISABLE_COPY(QQuickMouseArea)
    Q_DECLARE_PRIVATE(QQuickMouseArea)
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickMouseArea)

#endif // QQUICKMOUSEAREA_P_H
