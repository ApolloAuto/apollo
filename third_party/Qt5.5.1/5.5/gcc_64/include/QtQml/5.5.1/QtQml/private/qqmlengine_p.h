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

#ifndef QQMLENGINE_P_H
#define QQMLENGINE_P_H

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

#include "qqmlengine.h"

#include "qqmltypeloader_p.h"
#include "qqmlimport_p.h"
#include <private/qpodvector_p.h>
#include "qqml.h"
#include "qqmlvaluetype_p.h"
#include "qqmlcontext.h"
#include "qqmlcontext_p.h"
#include "qqmlexpression.h"
#include "qqmlproperty_p.h"
#include "qqmlpropertycache_p.h"
#include "qqmlmetatype_p.h"
#include "qqmldirparser_p.h"
#include <private/qintrusivelist_p.h>
#include <private/qrecyclepool_p.h>
#include <private/qfieldlist_p.h>

#include <QtCore/qlist.h>
#include <QtCore/qpair.h>
#include <QtCore/qstack.h>
#include <QtCore/qmutex.h>
#include <QtCore/qstring.h>
#include <QtCore/qthread.h>

#include <private/qobject_p.h>

#include <private/qv8engine_p.h>
#include <private/qjsengine_p.h>

QT_BEGIN_NAMESPACE

class QQmlContext;
class QQmlEngine;
class QQmlContextPrivate;
class QQmlExpression;
class QQmlImportDatabase;
class QNetworkReply;
class QNetworkAccessManager;
class QQmlNetworkAccessManagerFactory;
class QQmlAbstractBinding;
class QQmlTypeNameCache;
class QQmlComponentAttached;
class QQmlCleanup;
class QQmlDelayedError;
class QQuickWorkerScriptEngine;
class QQmlObjectCreator;
class QDir;
class QQmlIncubator;
class QQmlProfiler;

// This needs to be declared here so that the pool for it can live in QQmlEnginePrivate.
// The inline method definitions are in qqmljavascriptexpression_p.h
class QQmlJavaScriptExpressionGuard : public QQmlNotifierEndpoint
{
public:
    inline QQmlJavaScriptExpressionGuard(QQmlJavaScriptExpression *);

    static inline QQmlJavaScriptExpressionGuard *New(QQmlJavaScriptExpression *e,
                                                             QQmlEngine *engine);
    inline void Delete();

    QQmlJavaScriptExpression *expression;
    QQmlJavaScriptExpressionGuard *next;
};

class Q_QML_PRIVATE_EXPORT QQmlEnginePrivate : public QJSEnginePrivate
{
    Q_DECLARE_PUBLIC(QQmlEngine)
public:
    QQmlEnginePrivate(QQmlEngine *);
    ~QQmlEnginePrivate();

    void init();
    // No mutex protecting baseModulesUninitialized, because use outside QQmlEngine
    // is just qmlClearTypeRegistrations (which can't be called while an engine exists)
    static bool baseModulesUninitialized;

    class PropertyCapture {
    public:
        inline virtual ~PropertyCapture() {}
        virtual void captureProperty(QQmlNotifier *) = 0;
        virtual void captureProperty(QObject *, int, int) = 0;
    };

    PropertyCapture *propertyCapture;
    inline void captureProperty(QQmlNotifier *);
    inline void captureProperty(QObject *, int, int);

    QRecyclePool<QQmlJavaScriptExpressionGuard> jsExpressionGuardPool;

    QQmlContext *rootContext;
    bool isDebugging;
    QQmlProfiler *profiler;
    void enableProfiler();

    bool outputWarningsToMsgLog;

    // Registered cleanup handlers
    QQmlCleanup *cleanup;

    // Bindings that have had errors during startup
    QQmlDelayedError *erroredBindings;
    int inProgressCreations;

    QV8Engine *v8engine() const { return q_func()->handle(); }
    QV4::ExecutionEngine *v4engine() const { return QV8Engine::getV4(q_func()->handle()); }

    QQuickWorkerScriptEngine *getWorkerScriptEngine();
    QQuickWorkerScriptEngine *workerScriptEngine;

    QUrl baseUrl;

    typedef QPair<QPointer<QObject>,int> FinalizeCallback;
    void registerFinalizeCallback(QObject *obj, int index);

    QQmlObjectCreator *activeObjectCreator;

    QNetworkAccessManager *createNetworkAccessManager(QObject *parent) const;
    QNetworkAccessManager *getNetworkAccessManager() const;
    mutable QNetworkAccessManager *networkAccessManager;
    mutable QQmlNetworkAccessManagerFactory *networkAccessManagerFactory;

    QHash<QString,QSharedPointer<QQmlImageProviderBase> > imageProviders;

    QQmlAbstractUrlInterceptor* urlInterceptor;

    int scarceResourcesRefCount;
    void referenceScarceResources();
    void dereferenceScarceResources();

    QQmlTypeLoader typeLoader;
    QQmlImportDatabase importDatabase;


    QString offlineStoragePath;

    mutable quint32 uniqueId;
    inline quint32 getUniqueId() const {
        return uniqueId++;
    }

    // Unfortunate workaround to avoid a circular dependency between
    // qqmlengine_p.h and qqmlincubator_p.h
    struct Incubator : public QSharedData {
        QIntrusiveListNode next;
        // Unfortunate workaround for MSVC
        QIntrusiveListNode nextWaitingFor;
    };
    QIntrusiveList<Incubator, &Incubator::next> incubatorList;
    unsigned int incubatorCount;
    QQmlIncubationController *incubationController;
    void incubate(QQmlIncubator &, QQmlContextData *);

    // These methods may be called from any thread
    inline bool isEngineThread() const;
    inline static bool isEngineThread(const QQmlEngine *);
    template<typename T>
    inline void deleteInEngineThread(T *);
    template<typename T>
    inline static void deleteInEngineThread(QQmlEngine *, T *);

    // These methods may be called from the loader thread
    inline QQmlPropertyCache *cache(QQmlType *, int, QQmlError &error);
    using QJSEnginePrivate::cache;

    // These methods may be called from the loader thread
    bool isQObject(int);
    QObject *toQObject(const QVariant &, bool *ok = 0) const;
    QQmlMetaType::TypeCategory typeCategory(int) const;
    bool isList(int) const;
    int listType(int) const;
    QQmlMetaObject rawMetaObjectForType(int) const;
    QQmlMetaObject metaObjectForType(int) const;
    QQmlPropertyCache *propertyCacheForType(int);
    QQmlPropertyCache *rawPropertyCacheForType(int);
    void registerInternalCompositeType(QQmlCompiledData *);
    void unregisterInternalCompositeType(QQmlCompiledData *);

    bool isTypeLoaded(const QUrl &url) const;
    bool isScriptLoaded(const QUrl &url) const;

    inline void setDebugChangesCache(const QHash<QUrl, QByteArray> &changes);
    inline QHash<QUrl, QByteArray> debugChangesCache();

    void sendQuit();
    void warning(const QQmlError &);
    void warning(const QList<QQmlError> &);
    void warning(QQmlDelayedError *);
    static void warning(QQmlEngine *, const QQmlError &);
    static void warning(QQmlEngine *, const QList<QQmlError> &);
    static void warning(QQmlEngine *, QQmlDelayedError *);
    static void warning(QQmlEnginePrivate *, const QQmlError &);
    static void warning(QQmlEnginePrivate *, const QList<QQmlError> &);

    inline static QV8Engine *getV8Engine(QQmlEngine *e);
    inline static QV4::ExecutionEngine *getV4Engine(QQmlEngine *e);
    inline static QQmlEnginePrivate *get(QQmlEngine *e);
    inline static const QQmlEnginePrivate *get(const QQmlEngine *e);
    inline static QQmlEnginePrivate *get(QQmlContext *c);
    inline static QQmlEnginePrivate *get(QQmlContextData *c);
    inline static QQmlEngine *get(QQmlEnginePrivate *p);
    inline static QQmlEnginePrivate *get(QV4::ExecutionEngine *e);

    static void registerBaseTypes(const char *uri, int versionMajor, int versionMinor);
    static void registerQtQuick2Types(const char *uri, int versionMajor, int versionMinor);
    static void defineQtQuick2Module();

    static bool designerMode();
    static void activateDesignerMode();

    static bool qml_debugging_enabled;

    mutable QMutex networkAccessManagerMutex;

private:
    // Must be called locked
    QQmlPropertyCache *createCache(QQmlType *, int, QQmlError &error);

    // These members must be protected by a QQmlEnginePrivate::Locker as they are required by
    // the threaded loader.  Only access them through their respective accessor methods.
    QHash<QPair<QQmlType *, int>, QQmlPropertyCache *> typePropertyCache;
    QHash<int, int> m_qmlLists;
    QHash<int, QQmlCompiledData *> m_compositeTypes;
    QHash<QUrl, QByteArray> debugChangesHash;
    static bool s_designerMode;

    // These members is protected by the full QQmlEnginePrivate::mutex mutex
    struct Deletable { Deletable():next(0) {} virtual ~Deletable() {} Deletable *next; };
    QFieldList<Deletable, &Deletable::next> toDeleteInEngineThread;
    void doDeleteInEngineThread();
};

/*!
Returns true if the calling thread is the QQmlEngine thread.
*/
bool QQmlEnginePrivate::isEngineThread() const
{
    Q_Q(const QQmlEngine);
    return QThread::currentThread() == q->thread();
}

/*!
Returns true if the calling thread is the QQmlEngine \a engine thread.
*/
bool QQmlEnginePrivate::isEngineThread(const QQmlEngine *engine)
{
    Q_ASSERT(engine);
    return QQmlEnginePrivate::get(engine)->isEngineThread();
}

/*!
Delete \a value in the engine thread.  If the calling thread is the engine
thread, \a value will be deleted immediately.

This method should be used for *any* type that has resources that need to
be freed in the engine thread.  This is generally types that use V8 handles.
As there is some small overhead in checking the current thread, it is best
practice to check if any V8 handles actually need to be freed and delete
the instance directly if not.
*/
template<typename T>
void QQmlEnginePrivate::deleteInEngineThread(T *value)
{
    Q_Q(QQmlEngine);

    Q_ASSERT(value);
    if (isEngineThread()) {
        delete value;
    } else {
        struct I : public Deletable {
            I(T *value) : value(value) {}
            ~I() { delete value; }
            T *value;
        };
        I *i = new I(value);
        mutex.lock();
        bool wasEmpty = toDeleteInEngineThread.isEmpty();
        toDeleteInEngineThread.append(i);
        mutex.unlock();
        if (wasEmpty)
            QCoreApplication::postEvent(q, new QEvent(QEvent::User));
    }
}

/*!
Delete \a value in the \a engine thread.  If the calling thread is the engine
thread, \a value will be deleted immediately.
*/
template<typename T>
void QQmlEnginePrivate::deleteInEngineThread(QQmlEngine *engine, T *value)
{
    Q_ASSERT(engine);
    QQmlEnginePrivate::get(engine)->deleteInEngineThread<T>(value);
}

/*!
Returns a QQmlPropertyCache for \a type with \a minorVersion.

The returned cache is not referenced, so if it is to be stored, call addref().
*/
QQmlPropertyCache *QQmlEnginePrivate::cache(QQmlType *type, int minorVersion, QQmlError &error)
{
    Q_ASSERT(type);

    if (minorVersion == -1 || !type->containsRevisionedAttributes())
        return cache(type->metaObject());

    Locker locker(this);
    QQmlPropertyCache *rv = typePropertyCache.value(qMakePair(type, minorVersion));
    if (!rv) rv = createCache(type, minorVersion, error);
    return rv;
}

QV8Engine *QQmlEnginePrivate::getV8Engine(QQmlEngine *e)
{
    Q_ASSERT(e);

    return e->d_func()->v8engine();
}

QV4::ExecutionEngine *QQmlEnginePrivate::getV4Engine(QQmlEngine *e)
{
    Q_ASSERT(e);

    return e->d_func()->v4engine();
}

QQmlEnginePrivate *QQmlEnginePrivate::get(QQmlEngine *e)
{
    Q_ASSERT(e);

    return e->d_func();
}

const QQmlEnginePrivate *QQmlEnginePrivate::get(const QQmlEngine *e)
{
    Q_ASSERT(e);

    return e->d_func();
}

QQmlEnginePrivate *QQmlEnginePrivate::get(QQmlContext *c)
{
    return (c && c->engine()) ? QQmlEnginePrivate::get(c->engine()) : 0;
}

QQmlEnginePrivate *QQmlEnginePrivate::get(QQmlContextData *c)
{
    return (c && c->engine) ? QQmlEnginePrivate::get(c->engine) : 0;
}

QQmlEngine *QQmlEnginePrivate::get(QQmlEnginePrivate *p)
{
    Q_ASSERT(p);

    return p->q_func();
}

QQmlEnginePrivate *QQmlEnginePrivate::get(QV4::ExecutionEngine *e)
{
    if (!e->v8Engine)
        return 0;
    QQmlEngine *qmlEngine = e->v8Engine->engine();
    if (!qmlEngine)
        return 0;
    return get(qmlEngine);
}

void QQmlEnginePrivate::captureProperty(QQmlNotifier *n)
{
    if (propertyCapture)
        propertyCapture->captureProperty(n);
}

void QQmlEnginePrivate::captureProperty(QObject *o, int c, int n)
{
    if (propertyCapture)
        propertyCapture->captureProperty(o, c, n);
}

void QQmlEnginePrivate::setDebugChangesCache(const QHash<QUrl, QByteArray> &changes)
{
    Locker locker(this);
    foreach (const QUrl &key, changes.keys())
        debugChangesHash.insert(key, changes.value(key));
}

QHash<QUrl, QByteArray> QQmlEnginePrivate::debugChangesCache()
{
    Locker locker(this);
    return debugChangesHash;
}

QT_END_NAMESPACE

#endif // QQMLENGINE_P_H
