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

#ifndef QQMLCONTEXTWRAPPER_P_H
#define QQMLCONTEXTWRAPPER_P_H

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
#include <private/qtqmlglobal_p.h>

#include <private/qv4value_inl_p.h>
#include <private/qv4object_p.h>
#include <private/qqmlcontext_p.h>
#include <private/qv4functionobject_p.h>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace CompiledData {
struct Function;
}

struct QmlContextWrapper;

namespace Heap {

struct QQmlIdObjectsArray;

struct QmlContextWrapper : Object {
    QmlContextWrapper(ExecutionEngine *engine, QQmlContextData *context, QObject *scopeObject, bool ownsContext = false);
    ~QmlContextWrapper();
    bool readOnly;
    bool ownsContext;
    bool isNullWrapper;

    QQmlGuardedContextData context;
    QPointer<QObject> scopeObject;
    QQmlIdObjectsArray *idObjectsWrapper;
};

struct QQmlIdObjectsArray : Object {
    QQmlIdObjectsArray(QV4::ExecutionEngine *engine, QV4::QmlContextWrapper *contextWrapper);
    QmlContextWrapper *contextWrapper;
};

}

struct Q_QML_EXPORT QmlContextWrapper : Object
{
    V4_OBJECT2(QmlContextWrapper, Object)
    V4_NEEDS_DESTROY

    static ReturnedValue qmlScope(ExecutionEngine *e, QQmlContextData *ctxt, QObject *scope);
    static ReturnedValue urlScope(ExecutionEngine *v4, const QUrl &);

    static QQmlContextData *callingContext(ExecutionEngine *v4);
    static void takeContextOwnership(const Value &qmlglobal);

    inline QObject *getScopeObject() const { return d()->scopeObject; }
    inline QQmlContextData *getContext() const { return d()->context; }
    static QQmlContextData *getContext(const Value &value);

    void setReadOnly(bool b) { d()->readOnly = b; }

    static ReturnedValue get(Managed *m, String *name, bool *hasProperty);
    static void put(Managed *m, String *name, const Value &value);
    static void markObjects(Heap::Base *m, ExecutionEngine *engine);

    static void registerQmlDependencies(ExecutionEngine *context, const CompiledData::Function *compiledFunction);

    ReturnedValue idObjectsArray();
    ReturnedValue qmlSingletonWrapper(ExecutionEngine *e, String *name);

};

struct QQmlIdObjectsArray : public Object
{
    V4_OBJECT2(QQmlIdObjectsArray, Object)

    static ReturnedValue getIndexed(Managed *m, uint index, bool *hasProperty);
    static void markObjects(Heap::Base *that, ExecutionEngine *engine);

};

}

QT_END_NAMESPACE

#endif // QV8CONTEXTWRAPPER_P_H

