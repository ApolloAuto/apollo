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

#ifndef QJSENGINE_P_H
#define QJSENGINE_P_H

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

#include <QtCore/private/qobject_p.h>
#include <QtCore/qmutex.h>
#include "qjsengine.h"
#include "private/qtqmlglobal_p.h"

QT_BEGIN_NAMESPACE

class QQmlPropertyCache;

namespace QV4 {
struct ExecutionEngine;
}

class Q_QML_PRIVATE_EXPORT QJSEnginePrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QJSEngine)

public:
    static QJSEnginePrivate* get(QJSEngine*e) { return e->d_func(); }
    static const QJSEnginePrivate* get(const QJSEngine*e) { return e->d_func(); }
    static QJSEnginePrivate* get(QV4::ExecutionEngine *e);

    QJSEnginePrivate() : mutex(QMutex::Recursive) {}
    ~QJSEnginePrivate();

    // Locker locks the QQmlEnginePrivate data structures for read and write, if necessary.
    // Currently, locking is only necessary if the threaded loader is running concurrently.  If it is
    // either idle, or is running with the main thread blocked, no locking is necessary.  This way
    // we only pay for locking when we have to.
    // Consequently, this class should only be used to protect simple accesses or modifications of the
    // QQmlEnginePrivate structures or operations that can be guaranteed not to start activity
    // on the loader thread.
    // The Locker API is identical to QMutexLocker.  Locker reuses the QQmlEnginePrivate::mutex
    // QMutex instance and multiple Lockers are recursive in the same thread.
    class Locker
    {
    public:
        inline Locker(const QJSEngine *);
        inline Locker(const QJSEnginePrivate *);
        inline ~Locker();

        inline void unlock();
        inline void relock();

    private:
        const QJSEnginePrivate *m_ep;
        quint32 m_locked:1;
    };

    // Shared by QQmlEngine
    mutable QMutex mutex;


    // These methods may be called from the QML loader thread
    inline QQmlPropertyCache *cache(QObject *obj);
    inline QQmlPropertyCache *cache(const QMetaObject *);

private:
    // Must be called locked
    QQmlPropertyCache *createCache(const QMetaObject *);

    // These members must be protected by a QJSEnginePrivate::Locker as they are required by
    // the threaded loader.  Only access them through their respective accessor methods.
    QHash<const QMetaObject *, QQmlPropertyCache *> propertyCache;
};

QJSEnginePrivate::Locker::Locker(const QJSEngine *e)
: m_ep(QJSEnginePrivate::get(e))
{
    relock();
}

QJSEnginePrivate::Locker::Locker(const QJSEnginePrivate *e)
: m_ep(e), m_locked(false)
{
    relock();
}

QJSEnginePrivate::Locker::~Locker()
{
    unlock();
}

void QJSEnginePrivate::Locker::unlock()
{
    if (m_locked) {
        m_ep->mutex.unlock();
        m_locked = false;
    }
}

void QJSEnginePrivate::Locker::relock()
{
    Q_ASSERT(!m_locked);
    m_ep->mutex.lock();
    m_locked = true;
}

/*!
Returns a QQmlPropertyCache for \a obj if one is available.

If \a obj is null, being deleted or contains a dynamic meta object 0
is returned.

The returned cache is not referenced, so if it is to be stored, call addref().

XXX thread There is a potential future race condition in this and all the cache()
functions.  As the QQmlPropertyCache is returned unreferenced, when called
from the loader thread, it is possible that the cache will have been dereferenced
and deleted before the loader thread has a chance to use or reference it.  This
can't currently happen as the cache holds a reference to the
QQmlPropertyCache until the QQmlEngine is destroyed.
*/
QQmlPropertyCache *QJSEnginePrivate::cache(QObject *obj)
{
    if (!obj || QObjectPrivate::get(obj)->metaObject || QObjectPrivate::get(obj)->wasDeleted)
        return 0;

    Locker locker(this);
    const QMetaObject *mo = obj->metaObject();
    QQmlPropertyCache *rv = propertyCache.value(mo);
    if (!rv) rv = createCache(mo);
    return rv;
}

/*!
Returns a QQmlPropertyCache for \a metaObject.

As the cache is persisted for the life of the engine, \a metaObject must be
a static "compile time" meta-object, or a meta-object that is otherwise known to
exist for the lifetime of the QQmlEngine.

The returned cache is not referenced, so if it is to be stored, call addref().
*/
QQmlPropertyCache *QJSEnginePrivate::cache(const QMetaObject *metaObject)
{
    Q_ASSERT(metaObject);

    Locker locker(this);
    QQmlPropertyCache *rv = propertyCache.value(metaObject);
    if (!rv) rv = createCache(metaObject);
    return rv;
}


QT_END_NAMESPACE

#endif // QJSENGINE_P_H
