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

#ifndef QQUICKEVENTS_P_P_H
#define QQUICKEVENTS_P_P_H

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

#include <private/qtquickglobal_p.h>
#include <qqml.h>

#include <QtCore/qobject.h>
#include <QtGui/qvector2d.h>
#include <QtGui/qevent.h>
#include <QtGui/qkeysequence.h>

QT_BEGIN_NAMESPACE

class QQuickKeyEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int key READ key)
    Q_PROPERTY(QString text READ text)
    Q_PROPERTY(int modifiers READ modifiers)
    Q_PROPERTY(bool isAutoRepeat READ isAutoRepeat)
    Q_PROPERTY(int count READ count)
    Q_PROPERTY(quint32 nativeScanCode READ nativeScanCode)
    Q_PROPERTY(bool accepted READ isAccepted WRITE setAccepted)

public:
    QQuickKeyEvent(QEvent::Type type, int key, Qt::KeyboardModifiers modifiers, const QString &text=QString(), bool autorep=false, ushort count=1)
        : event(type, key, modifiers, text, autorep, count) { event.setAccepted(false); }
    QQuickKeyEvent(const QKeyEvent &ke)
        : event(ke) { event.setAccepted(false); }

    int key() const { return event.key(); }
    QString text() const { return event.text(); }
    int modifiers() const { return event.modifiers(); }
    bool isAutoRepeat() const { return event.isAutoRepeat(); }
    int count() const { return event.count(); }
    quint32 nativeScanCode() const { return event.nativeScanCode(); }

    bool isAccepted() { return event.isAccepted(); }
    void setAccepted(bool accepted) { event.setAccepted(accepted); }

    Q_REVISION(2) Q_INVOKABLE bool matches(QKeySequence::StandardKey key) const { return event.matches(key); }

private:
    QKeyEvent event;
};

// used in Qt Location
class Q_QUICK_PRIVATE_EXPORT QQuickMouseEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x)
    Q_PROPERTY(qreal y READ y)
    Q_PROPERTY(int button READ button)
    Q_PROPERTY(int buttons READ buttons)
    Q_PROPERTY(int modifiers READ modifiers)
    Q_PROPERTY(bool wasHeld READ wasHeld)
    Q_PROPERTY(bool isClick READ isClick)
    Q_PROPERTY(bool accepted READ isAccepted WRITE setAccepted)

public:
    QQuickMouseEvent(qreal x, qreal y, Qt::MouseButton button, Qt::MouseButtons buttons, Qt::KeyboardModifiers modifiers
                  , bool isClick=false, bool wasHeld=false)
        : _x(x), _y(y), _button(button), _buttons(buttons), _modifiers(modifiers)
          , _wasHeld(wasHeld), _isClick(isClick), _accepted(true) {}

    qreal x() const { return _x; }
    qreal y() const { return _y; }
    int button() const { return _button; }
    int buttons() const { return _buttons; }
    int modifiers() const { return _modifiers; }
    bool wasHeld() const { return _wasHeld; }
    bool isClick() const { return _isClick; }

    // only for internal usage
    void setX(qreal x) { _x = x; }
    void setY(qreal y) { _y = y; }
    void setPosition(const QPointF &point) { _x = point.x(); _y = point.y(); }

    bool isAccepted() { return _accepted; }
    void setAccepted(bool accepted) { _accepted = accepted; }

private:
    qreal _x;
    qreal _y;
    Qt::MouseButton _button;
    Qt::MouseButtons _buttons;
    Qt::KeyboardModifiers _modifiers;
    bool _wasHeld;
    bool _isClick;
    bool _accepted;
};

class QQuickWheelEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x)
    Q_PROPERTY(qreal y READ y)
    Q_PROPERTY(QPoint angleDelta READ angleDelta)
    Q_PROPERTY(QPoint pixelDelta READ pixelDelta)
    Q_PROPERTY(int buttons READ buttons)
    Q_PROPERTY(int modifiers READ modifiers)
    Q_PROPERTY(bool accepted READ isAccepted WRITE setAccepted)

public:
    QQuickWheelEvent(qreal x, qreal y, const QPoint& angleDelta, const QPoint& pixelDelta,
                     Qt::MouseButtons buttons, Qt::KeyboardModifiers modifiers)
        : _x(x), _y(y), _angleDelta(angleDelta), _pixelDelta(pixelDelta), _buttons(buttons),
          _modifiers(modifiers), _accepted(true) {}

    qreal x() const { return _x; }
    qreal y() const { return _y; }
    QPoint angleDelta() const { return _angleDelta; }
    QPoint pixelDelta() const { return _pixelDelta; }
    int buttons() const { return _buttons; }
    int modifiers() const { return _modifiers; }

    bool isAccepted() { return _accepted; }
    void setAccepted(bool accepted) { _accepted = accepted; }

private:
    qreal _x;
    qreal _y;
    QPoint _angleDelta;
    QPoint _pixelDelta;
    Qt::MouseButtons _buttons;
    Qt::KeyboardModifiers _modifiers;
    bool _accepted;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickKeyEvent)
QML_DECLARE_TYPE(QQuickMouseEvent)
QML_DECLARE_TYPE(QQuickWheelEvent)

#endif // QQUICKEVENTS_P_P_H
