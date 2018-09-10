/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQMLREFCOUNT_P_H
#define QQMLREFCOUNT_P_H

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

#include <QtCore/qglobal.h>
#include <QtCore/qatomic.h>
#include <private/qv4global_p.h>

QT_BEGIN_NAMESPACE


class Q_QML_PRIVATE_EXPORT QQmlRefCount
{
public:
    inline QQmlRefCount();
    inline virtual ~QQmlRefCount();
    inline void addref();
    inline void release();
    inline int count() const;

protected:
    inline virtual void destroy();

private:
    QAtomicInt refCount;
};

template<class T>
class QQmlRefPointer
{
public:
    inline QQmlRefPointer();
    inline QQmlRefPointer(T *);
    inline QQmlRefPointer(const QQmlRefPointer<T> &);
    inline ~QQmlRefPointer();

    inline QQmlRefPointer<T> &operator=(const QQmlRefPointer<T> &o);
    inline QQmlRefPointer<T> &operator=(T *);

    inline bool isNull() const { return !o; }

    inline T* operator->() const { return o; }
    inline T& operator*() const { return *o; }
    inline operator T*() const { return o; }
    inline T* data() const { return o; }

    inline QQmlRefPointer<T> &take(T *);

private:
    T *o;
};

QQmlRefCount::QQmlRefCount()
: refCount(1)
{
}

QQmlRefCount::~QQmlRefCount()
{
    Q_ASSERT(refCount.load() == 0);
}

void QQmlRefCount::addref()
{
    Q_ASSERT(refCount.load() > 0);
    refCount.ref();
}

void QQmlRefCount::release()
{
    Q_ASSERT(refCount.load() > 0);
    if (!refCount.deref())
        destroy();
}

int QQmlRefCount::count() const
{
    return refCount.load();
}

void QQmlRefCount::destroy()
{
    delete this;
}

template<class T>
QQmlRefPointer<T>::QQmlRefPointer()
: o(0)
{
}

template<class T>
QQmlRefPointer<T>::QQmlRefPointer(T *o)
: o(o)
{
    if (o) o->addref();
}

template<class T>
QQmlRefPointer<T>::QQmlRefPointer(const QQmlRefPointer<T> &other)
: o(other.o)
{
    if (o) o->addref();
}

template<class T>
QQmlRefPointer<T>::~QQmlRefPointer()
{
    if (o) o->release();
}

template<class T>
QQmlRefPointer<T> &QQmlRefPointer<T>::operator=(const QQmlRefPointer<T> &other)
{
    if (other.o) other.o->addref();
    if (o) o->release();
    o = other.o;
    return *this;
}

template<class T>
QQmlRefPointer<T> &QQmlRefPointer<T>::operator=(T *other)
{
    if (other) other->addref();
    if (o) o->release();
    o = other;
    return *this;
}

/*!
Takes ownership of \a other.  take() does *not* add a reference, as it assumes ownership
of the callers reference of other.
*/
template<class T>
QQmlRefPointer<T> &QQmlRefPointer<T>::take(T *other)
{
    if (o) o->release();
    o = other;
    return *this;
}

QT_END_NAMESPACE

#endif // QQMLREFCOUNT_P_H
