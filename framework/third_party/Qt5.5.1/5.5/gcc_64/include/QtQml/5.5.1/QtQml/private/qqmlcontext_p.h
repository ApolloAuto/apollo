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

#ifndef QQMLCONTEXT_P_H
#define QQMLCONTEXT_P_H

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

#include "qqmlcontext.h"

#include "qqmldata_p.h"
#include "qqmltypenamecache_p.h"
#include "qqmlnotifier_p.h"
#include "qqmllist.h"

#include <QtCore/qhash.h>
#include <QtQml/qjsvalue.h>
#include <QtCore/qset.h>

#include <private/qobject_p.h>
#include <private/qflagpointer_p.h>
#include <private/qqmlguard_p.h>

#include <private/qv4identifier_p.h>

QT_BEGIN_NAMESPACE

class QQmlContext;
class QQmlExpression;
class QQmlEngine;
class QQmlExpression;
class QQmlExpressionPrivate;
class QQmlAbstractExpression;
class QQmlContextData;

class QQmlContextPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QQmlContext)
public:
    QQmlContextPrivate();

    QQmlContextData *data;

    QList<QVariant> propertyValues;
    int notifyIndex;

    static QQmlContextPrivate *get(QQmlContext *context) {
        return static_cast<QQmlContextPrivate *>(QObjectPrivate::get(context));
    }
    static QQmlContext *get(QQmlContextPrivate *context) {
        return static_cast<QQmlContext *>(context->q_func());
    }

    // Only used for debugging
    QList<QPointer<QObject> > instances;

    static int context_count(QQmlListProperty<QObject> *);
    static QObject *context_at(QQmlListProperty<QObject> *, int);
};

class QQmlComponentAttached;
class QQmlGuardedContextData;
class Q_QML_PRIVATE_EXPORT QQmlContextData
{
public:
    QQmlContextData();
    QQmlContextData(QQmlContext *);
    void emitDestruction();
    void clearContext();
    void destroy();
    void invalidate();

    inline bool isValid() const {
        return engine && (!isInternal || !contextObject || !QObjectPrivate::get(contextObject)->wasDeleted);
    }

    // My parent context and engine
    QQmlContextData *parent;
    QQmlEngine *engine;

    void setParent(QQmlContextData *, bool parentTakesOwnership = false);
    void refreshExpressions();

    void addObject(QObject *);

    QUrl resolvedUrl(const QUrl &);

    // My containing QQmlContext.  If isInternal is true this owns publicContext.
    // If internal is false publicContext owns this.
    QQmlContext *asQQmlContext();
    QQmlContextPrivate *asQQmlContextPrivate();
    quint32 isInternal:1;
    quint32 ownedByParent:1; // unrelated to isInternal; parent context deletes children if true.
    quint32 isJSContext:1;
    quint32 isPragmaLibraryContext:1;
    quint32 unresolvedNames:1; // True if expressions in this context failed to resolve a toplevel name
    quint32 hasEmittedDestruction:1;
    quint32 isRootObjectInCreation:1;
    quint32 dummy:25;
    QQmlContext *publicContext;

    // VME data that is constructing this context if any
    void *activeVMEData;

    // Compilation unit for contexts that belong to a compiled type.
    QQmlRefPointer<QV4::CompiledData::CompilationUnit> typeCompilationUnit;

    mutable QHash<int, int> objectIndexToId;
    mutable QV4::IdentifierHash<int> propertyNameCache;
    QV4::IdentifierHash<int> &propertyNames() const;

    // Context object
    QObject *contextObject;

    // Any script blocks that exist on this context
    QV4::PersistentValue importedScripts; // This is a JS Array

    QUrl baseUrl;
    QString baseUrlString;

    QUrl url() const;
    QString urlString() const;

    // List of imports that apply to this context
    QQmlTypeNameCache *imports;

    // My children
    QQmlContextData *childContexts;

    // My peers in parent's childContexts list
    QQmlContextData  *nextChild;
    QQmlContextData **prevChild;

    // Expressions that use this context
    QQmlAbstractExpression *expressions;

    // Doubly-linked list of objects that are owned by this context
    QQmlData *contextObjects;

    // Doubly-linked list of context guards (XXX merge with contextObjects)
    QQmlGuardedContextData *contextGuards;

    // id guards
    struct ContextGuard : public QQmlGuard<QObject>
    {
        inline ContextGuard();
        inline ContextGuard &operator=(QObject *obj);
        inline void objectDestroyed(QObject *);

        inline bool wasSet() const;

        QFlagPointer<QQmlContextData> context;
        QQmlNotifier bindings;
    };
    ContextGuard *idValues;
    int idValueCount;
    void setIdProperty(int, QObject *);
    void setIdPropertyData(const QHash<int, int> &);

    // Linked contexts. this owns linkedContext.
    QQmlContextData *linkedContext;

    // Linked list of uses of the Component attached property in this
    // context
    QQmlComponentAttached *componentAttached;

    // Return the outermost id for obj, if any.
    QString findObjectId(const QObject *obj) const;

    static QQmlContextData *get(QQmlContext *context) {
        return QQmlContextPrivate::get(context)->data;
    }

private:
    void refreshExpressionsRecursive(bool isGlobal);
    void refreshExpressionsRecursive(QQmlAbstractExpression *);
    ~QQmlContextData() {}
};

class QQmlGuardedContextData
{
public:
    inline QQmlGuardedContextData();
    inline QQmlGuardedContextData(QQmlContextData *);
    inline ~QQmlGuardedContextData();

    inline QQmlContextData *contextData();
    inline void setContextData(QQmlContextData *);

    inline bool isNull() const { return !m_contextData; }

    inline operator QQmlContextData*() const { return m_contextData; }
    inline QQmlContextData* operator->() const { return m_contextData; }
    inline QQmlGuardedContextData &operator=(QQmlContextData *d);

private:
    QQmlGuardedContextData &operator=(const QQmlGuardedContextData &);
    QQmlGuardedContextData(const QQmlGuardedContextData &);
    friend class QQmlContextData;

    inline void clear();

    QQmlContextData *m_contextData;
    QQmlGuardedContextData  *m_next;
    QQmlGuardedContextData **m_prev;
};

QQmlGuardedContextData::QQmlGuardedContextData()
: m_contextData(0), m_next(0), m_prev(0)
{
}

QQmlGuardedContextData::QQmlGuardedContextData(QQmlContextData *data)
: m_contextData(0), m_next(0), m_prev(0)
{
    setContextData(data);
}

QQmlGuardedContextData::~QQmlGuardedContextData()
{
    clear();
}

void QQmlGuardedContextData::setContextData(QQmlContextData *contextData)
{
    clear();

    if (contextData) {
        m_contextData = contextData;
        m_next = contextData->contextGuards;
        if (m_next) m_next->m_prev = &m_next;
        m_prev = &contextData->contextGuards;
        contextData->contextGuards = this;
    }
}

QQmlContextData *QQmlGuardedContextData::contextData()
{
    return m_contextData;
}

void QQmlGuardedContextData::clear()
{
    if (m_prev) {
        *m_prev = m_next;
        if (m_next) m_next->m_prev = m_prev;
        m_contextData = 0;
        m_next = 0;
        m_prev = 0;
    }
}

QQmlGuardedContextData &
QQmlGuardedContextData::operator=(QQmlContextData *d)
{
    setContextData(d);
    return *this;
}

QQmlContextData::ContextGuard::ContextGuard()
: context(0)
{
}

QQmlContextData::ContextGuard &QQmlContextData::ContextGuard::operator=(QObject *obj)
{
    QQmlGuard<QObject>::operator=(obj);
    context.setFlag();
    bindings.notify(); // For alias connections
    return *this;
}

void QQmlContextData::ContextGuard::objectDestroyed(QObject *)
{
    if (context->contextObject && !QObjectPrivate::get(context->contextObject)->wasDeleted)
        bindings.notify();
}

bool QQmlContextData::ContextGuard::wasSet() const
{
    return context.flag();
}

QT_END_NAMESPACE

#endif // QQMLCONTEXT_P_H
