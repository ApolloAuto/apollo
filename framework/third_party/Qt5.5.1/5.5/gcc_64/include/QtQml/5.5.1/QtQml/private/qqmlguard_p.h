/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#ifndef QQMLGUARD_P_H
#define QQMLGUARD_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of qapplication_*.cpp, qwidget*.cpp and qfiledialog.cpp.  This header
// file may change from version to version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qglobal.h>
#include <QtCore/qvariant.h>
#include <private/qqmldata_p.h>

QT_BEGIN_NAMESPACE

class QQmlGuardImpl
{
public:
    inline QQmlGuardImpl();
    inline QQmlGuardImpl(QObject *);
    inline QQmlGuardImpl(const QQmlGuardImpl &);
    inline ~QQmlGuardImpl();

    QObject *o;
    QQmlGuardImpl  *next;
    QQmlGuardImpl **prev;

    inline void addGuard();
    inline void remGuard();
};

class QObject;
template<class T>
class QQmlGuard : private QQmlGuardImpl
{
    friend class QQmlData;
public:
    inline QQmlGuard();
    inline QQmlGuard(T *);
    inline QQmlGuard(const QQmlGuard<T> &);
    inline virtual ~QQmlGuard();

    inline QQmlGuard<T> &operator=(const QQmlGuard<T> &o);
    inline QQmlGuard<T> &operator=(T *);

    inline T *object() const;
    inline void setObject(T *g);

    inline bool isNull() const
        { return !o; }

    inline T* operator->() const
        { return static_cast<T*>(const_cast<QObject*>(o)); }
    inline T& operator*() const
        { return *static_cast<T*>(const_cast<QObject*>(o)); }
    inline operator T*() const
        { return static_cast<T*>(const_cast<QObject*>(o)); }
    inline T* data() const
        { return static_cast<T*>(const_cast<QObject*>(o)); }

protected:
    virtual void objectDestroyed(T *) {}
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QQmlGuard<QObject>)

QT_BEGIN_NAMESPACE

QQmlGuardImpl::QQmlGuardImpl()
: o(0), next(0), prev(0)
{
}

QQmlGuardImpl::QQmlGuardImpl(QObject *g)
: o(g), next(0), prev(0)
{
    if (o) addGuard();
}

QQmlGuardImpl::QQmlGuardImpl(const QQmlGuardImpl &g)
: o(g.o), next(0), prev(0)
{
    if (o) addGuard();
}

QQmlGuardImpl::~QQmlGuardImpl()
{
    if (prev) remGuard();
    o = 0;
}

void QQmlGuardImpl::addGuard()
{
    Q_ASSERT(!prev);

    if (QObjectPrivate::get(o)->wasDeleted)
        return;

    QQmlData *data = QQmlData::get(o, true);
    next = data->guards;
    if (next) next->prev = &next;
    data->guards = this;
    prev = &data->guards;
}

void QQmlGuardImpl::remGuard()
{
    Q_ASSERT(prev);

    if (next) next->prev = prev;
    *prev = next;
    next = 0;
    prev = 0;
}

template<class T>
QQmlGuard<T>::QQmlGuard()
{
}

template<class T>
QQmlGuard<T>::QQmlGuard(T *g)
: QQmlGuardImpl(g)
{
}

template<class T>
QQmlGuard<T>::QQmlGuard(const QQmlGuard<T> &g)
: QQmlGuardImpl(g)
{
}

template<class T>
QQmlGuard<T>::~QQmlGuard()
{
}

template<class T>
QQmlGuard<T> &QQmlGuard<T>::operator=(const QQmlGuard<T> &g)
{
    setObject(g.object());
    return *this;
}

template<class T>
QQmlGuard<T> &QQmlGuard<T>::operator=(T *g)
{
    setObject(g);
    return *this;
}

template<class T>
T *QQmlGuard<T>::object() const
{
    return static_cast<T *>(o);
}

template<class T>
void QQmlGuard<T>::setObject(T *g)
{
    if (g != o) {
        if (prev) remGuard();
        o = g;
        if (o) addGuard();
    }
}

QT_END_NAMESPACE

#endif // QQMLGUARD_P_H
