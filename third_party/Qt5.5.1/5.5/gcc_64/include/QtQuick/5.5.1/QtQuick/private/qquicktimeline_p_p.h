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

#ifndef QQUICKTIMELINE_H
#define QQUICKTIMELINE_H

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

#include <QtCore/QObject>
#include "private/qabstractanimationjob_p.h"

QT_BEGIN_NAMESPACE

class QEasingCurve;
class QQuickTimeLineValue;
class QQuickTimeLineCallback;
struct QQuickTimeLinePrivate;
class QQuickTimeLineObject;
class Q_AUTOTEST_EXPORT QQuickTimeLine : public QObject, QAbstractAnimationJob
{
Q_OBJECT
public:
    QQuickTimeLine(QObject *parent = 0);
    ~QQuickTimeLine();

    enum SyncMode { LocalSync, GlobalSync };
    SyncMode syncMode() const;
    void setSyncMode(SyncMode);

    void pause(QQuickTimeLineObject &, int);
    void callback(const QQuickTimeLineCallback &);
    void set(QQuickTimeLineValue &, qreal);

    int accel(QQuickTimeLineValue &, qreal velocity, qreal accel);
    int accel(QQuickTimeLineValue &, qreal velocity, qreal accel, qreal maxDistance);
    int accelDistance(QQuickTimeLineValue &, qreal velocity, qreal distance);

    void move(QQuickTimeLineValue &, qreal destination, int time = 500);
    void move(QQuickTimeLineValue &, qreal destination, const QEasingCurve &, int time = 500);
    void moveBy(QQuickTimeLineValue &, qreal change, int time = 500);
    void moveBy(QQuickTimeLineValue &, qreal change, const QEasingCurve &, int time = 500);

    void sync();
    void setSyncPoint(int);
    int syncPoint() const;

    void sync(QQuickTimeLineValue &);
    void sync(QQuickTimeLineValue &, QQuickTimeLineValue &);

    void reset(QQuickTimeLineValue &);

    void complete();
    void clear();
    bool isActive() const;

    int time() const;

    virtual int duration() const;
Q_SIGNALS:
    void updated();
    void completed();

protected:
    virtual void updateCurrentTime(int);
    void debugAnimation(QDebug d) const;

private:
    void remove(QQuickTimeLineObject *);
    friend class QQuickTimeLineObject;
    friend struct QQuickTimeLinePrivate;
    QQuickTimeLinePrivate *d;
};

class Q_AUTOTEST_EXPORT QQuickTimeLineObject
{
public:
    QQuickTimeLineObject();
    virtual ~QQuickTimeLineObject();

protected:
    friend class QQuickTimeLine;
    friend struct QQuickTimeLinePrivate;
    QQuickTimeLine *_t;
};

class Q_AUTOTEST_EXPORT QQuickTimeLineValue : public QQuickTimeLineObject
{
public:
    QQuickTimeLineValue(qreal v = 0.) : _v(v) {}

    virtual qreal value() const { return _v; }
    virtual void setValue(qreal v) { _v = v; }

    QQuickTimeLine *timeLine() const { return _t; }

    operator qreal() const { return _v; }
    QQuickTimeLineValue &operator=(qreal v) { setValue(v); return *this; }
private:
    friend class QQuickTimeLine;
    friend struct QQuickTimeLinePrivate;
    qreal _v;
};

class Q_AUTOTEST_EXPORT QQuickTimeLineCallback
{
public:
    typedef void (*Callback)(void *);

    QQuickTimeLineCallback();
    QQuickTimeLineCallback(QQuickTimeLineObject *b, Callback, void * = 0);
    QQuickTimeLineCallback(const QQuickTimeLineCallback &o);

    QQuickTimeLineCallback &operator=(const QQuickTimeLineCallback &o);
    QQuickTimeLineObject *callbackObject() const;

private:
    friend struct QQuickTimeLinePrivate;
    Callback d0;
    void *d1;
    QQuickTimeLineObject *d2;
};

template<class T>
class QQuickTimeLineValueProxy : public QQuickTimeLineValue
{
public:
    QQuickTimeLineValueProxy(T *cls, void (T::*func)(qreal), qreal v = 0.)
    : QQuickTimeLineValue(v), _class(cls), _setFunctionReal(func), _setFunctionInt(0)
    {
        Q_ASSERT(_class);
    }

    QQuickTimeLineValueProxy(T *cls, void (T::*func)(int), qreal v = 0.)
    : QQuickTimeLineValue(v), _class(cls), _setFunctionReal(0), _setFunctionInt(func)
    {
        Q_ASSERT(_class);
    }

    virtual void setValue(qreal v)
    {
        QQuickTimeLineValue::setValue(v);
        if (_setFunctionReal) (_class->*_setFunctionReal)(v);
        else if (_setFunctionInt) (_class->*_setFunctionInt)((int)v);
    }

private:
    T *_class;
    void (T::*_setFunctionReal)(qreal);
    void (T::*_setFunctionInt)(int);
};

QT_END_NAMESPACE

#endif
