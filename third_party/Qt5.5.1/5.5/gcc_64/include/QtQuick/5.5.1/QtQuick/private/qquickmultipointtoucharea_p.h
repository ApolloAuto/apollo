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

#ifndef QQUICKMULTIPOINTTOUCHAREA_H
#define QQUICKMULTIPOINTTOUCHAREA_H

#include "qquickitem.h"
#include "qevent.h"

#include <QPointer>
#include <QMap>
#include <QList>
#include <QtGui/qguiapplication.h>
#include <QtGui/qstylehints.h>

QT_BEGIN_NAMESPACE

class QQuickMultiPointTouchArea;
class Q_AUTOTEST_EXPORT QQuickTouchPoint : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int pointId READ pointId NOTIFY pointIdChanged)
    Q_PROPERTY(bool pressed READ pressed NOTIFY pressedChanged)
    Q_PROPERTY(qreal x READ x NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y NOTIFY yChanged)
    Q_PROPERTY(qreal pressure READ pressure NOTIFY pressureChanged)
    Q_PROPERTY(QVector2D velocity READ velocity NOTIFY velocityChanged)
    Q_PROPERTY(QRectF area READ area NOTIFY areaChanged)

    Q_PROPERTY(qreal startX READ startX NOTIFY startXChanged)
    Q_PROPERTY(qreal startY READ startY NOTIFY startYChanged)
    Q_PROPERTY(qreal previousX READ previousX NOTIFY previousXChanged)
    Q_PROPERTY(qreal previousY READ previousY NOTIFY previousYChanged)
    Q_PROPERTY(qreal sceneX READ sceneX NOTIFY sceneXChanged)
    Q_PROPERTY(qreal sceneY READ sceneY NOTIFY sceneYChanged)

public:
    QQuickTouchPoint(bool qmlDefined = true)
        : _id(0),
          _x(0.0), _y(0.0),
          _pressure(0.0),
          _qmlDefined(qmlDefined),
          _inUse(false),
          _pressed(false),
          _previousX(0.0), _previousY(0.0),
          _sceneX(0.0), _sceneY(0.0)
    {}

    int pointId() const { return _id; }
    void setPointId(int id);

    qreal x() const { return _x; }
    void setX(qreal x);

    qreal y() const { return _y; }
    void setY(qreal y);

    qreal pressure() const { return _pressure; }
    void setPressure(qreal pressure);

    QVector2D velocity() const { return _velocity; }
    void setVelocity(const QVector2D &velocity);

    QRectF area() const { return _area; }
    void setArea(const QRectF &area);

    bool isQmlDefined() const { return _qmlDefined; }

    bool inUse() const { return _inUse; }
    void setInUse(bool inUse) { _inUse = inUse; }

    bool pressed() const { return _pressed; }
    void setPressed(bool pressed);

    qreal startX() const { return _startX; }
    void setStartX(qreal startX);

    qreal startY() const { return _startY; }
    void setStartY(qreal startY);

    qreal previousX() const { return _previousX; }
    void setPreviousX(qreal previousX);

    qreal previousY() const { return _previousY; }
    void setPreviousY(qreal previousY);

    qreal sceneX() const { return _sceneX; }
    void setSceneX(qreal sceneX);

    qreal sceneY() const { return _sceneY; }
    void setSceneY(qreal sceneY);

Q_SIGNALS:
    void pressedChanged();
    void pointIdChanged();
    void xChanged();
    void yChanged();
    void pressureChanged();
    void velocityChanged();
    void areaChanged();
    void startXChanged();
    void startYChanged();
    void previousXChanged();
    void previousYChanged();
    void sceneXChanged();
    void sceneYChanged();

private:
    friend class QQuickMultiPointTouchArea;
    int _id;
    qreal _x;
    qreal _y;
    qreal _pressure;
    QVector2D _velocity;
    QRectF _area;
    bool _qmlDefined;
    bool _inUse;    //whether the point is currently in use (only valid when _qmlDefined == true)
    bool _pressed;
    qreal _startX;
    qreal _startY;
    qreal _previousX;
    qreal _previousY;
    qreal _sceneX;
    qreal _sceneY;
};

class QQuickGrabGestureEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QQmlListProperty<QObject> touchPoints READ touchPoints)
    Q_PROPERTY(qreal dragThreshold READ dragThreshold)
public:
    QQuickGrabGestureEvent() : _grab(false), _dragThreshold(QGuiApplication::styleHints()->startDragDistance()) {}

    Q_INVOKABLE void grab() { _grab = true; }
    bool wantsGrab() const { return _grab; }

    QQmlListProperty<QObject> touchPoints() {
        return QQmlListProperty<QObject>(this, _touchPoints);
    }
    qreal dragThreshold() const { return _dragThreshold; }

private:
    friend class QQuickMultiPointTouchArea;
    bool _grab;
    qreal _dragThreshold;
    QList<QObject*> _touchPoints;
};

class Q_AUTOTEST_EXPORT QQuickMultiPointTouchArea : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(QQmlListProperty<QQuickTouchPoint> touchPoints READ touchPoints)
    Q_PROPERTY(int minimumTouchPoints READ minimumTouchPoints WRITE setMinimumTouchPoints NOTIFY minimumTouchPointsChanged)
    Q_PROPERTY(int maximumTouchPoints READ maximumTouchPoints WRITE setMaximumTouchPoints NOTIFY maximumTouchPointsChanged)
    Q_PROPERTY(bool mouseEnabled READ mouseEnabled WRITE setMouseEnabled NOTIFY mouseEnabledChanged)

public:
    QQuickMultiPointTouchArea(QQuickItem *parent=0);
    ~QQuickMultiPointTouchArea();

    int minimumTouchPoints() const;
    void setMinimumTouchPoints(int num);
    int maximumTouchPoints() const;
    void setMaximumTouchPoints(int num);
    bool mouseEnabled() const { return _mouseEnabled; }
    void setMouseEnabled(bool arg);

    QQmlListProperty<QQuickTouchPoint> touchPoints() {
        return QQmlListProperty<QQuickTouchPoint>(this, 0, QQuickMultiPointTouchArea::touchPoint_append, QQuickMultiPointTouchArea::touchPoint_count, QQuickMultiPointTouchArea::touchPoint_at, 0);
    }

    static void touchPoint_append(QQmlListProperty<QQuickTouchPoint> *list, QQuickTouchPoint* touch) {
        QQuickMultiPointTouchArea *q = static_cast<QQuickMultiPointTouchArea*>(list->object);
        q->addTouchPrototype(touch);
    }

    static int touchPoint_count(QQmlListProperty<QQuickTouchPoint> *list) {
        QQuickMultiPointTouchArea *q = static_cast<QQuickMultiPointTouchArea*>(list->object);
        return q->_touchPrototypes.count();
    }

    static QQuickTouchPoint* touchPoint_at(QQmlListProperty<QQuickTouchPoint> *list, int index) {
        QQuickMultiPointTouchArea *q = static_cast<QQuickMultiPointTouchArea*>(list->object);
        return q->_touchPrototypes[index];
    }

Q_SIGNALS:
    void pressed(const QList<QObject*> &touchPoints);
    void updated(const QList<QObject*> &touchPoints);
    void released(const QList<QObject*> &touchPoints);
    void canceled(const QList<QObject*> &touchPoints);
    void gestureStarted(QQuickGrabGestureEvent *gesture);
    void touchUpdated(const QList<QObject*> &touchPoints);
    void minimumTouchPointsChanged();
    void maximumTouchPointsChanged();
    void mouseEnabledChanged();

protected:
    void touchEvent(QTouchEvent *) Q_DECL_OVERRIDE;
    bool childMouseEventFilter(QQuickItem *i, QEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseUngrabEvent() Q_DECL_OVERRIDE;
    void touchUngrabEvent() Q_DECL_OVERRIDE;

    void addTouchPrototype(QQuickTouchPoint* prototype);
    void addTouchPoint(const QTouchEvent::TouchPoint *p);
    void addTouchPoint(const QMouseEvent *e);
    void clearTouchLists();

    void updateTouchPoint(QQuickTouchPoint*, const QTouchEvent::TouchPoint*);
    void updateTouchPoint(QQuickTouchPoint *dtp, const QMouseEvent *e);
    void updateTouchData(QEvent*);

    bool sendMouseEvent(QMouseEvent *event);
    bool shouldFilter(QEvent *event);
    void grabGesture();
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;
#ifdef Q_OS_OSX
    void hoverEnterEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void hoverLeaveEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void setTouchEventsEnabled(bool enable);
#endif

private:
    void ungrab();
    QMap<int,QQuickTouchPoint*> _touchPrototypes;  //TouchPoints defined in QML
    QMap<int,QObject*> _touchPoints;            //All current touch points
    QList<QObject*> _releasedTouchPoints;
    QList<QObject*> _pressedTouchPoints;
    QList<QObject*> _movedTouchPoints;
    int _minimumTouchPoints;
    int _maximumTouchPoints;
    QPointer<QQuickTouchPoint> _mouseTouchPoint; // exists when mouse button is down and _mouseEnabled is true; null otherwise
    QTouchEvent::TouchPoint _mouseQpaTouchPoint; // synthetic QPA touch point to hold state and position of the mouse
    QPointF _mousePos;
    bool _stealMouse;
    bool _mouseEnabled;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickTouchPoint)
QML_DECLARE_TYPE(QQuickGrabGestureEvent)
QML_DECLARE_TYPE(QQuickMultiPointTouchArea)

#endif // QQUICKMULTIPOINTTOUCHAREA_H
