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
#ifndef QV4ECMAOBJECTS_P_H
#define QV4ECMAOBJECTS_P_H

#include "qv4object_p.h"
#include "qv4functionobject_p.h"
#include <QtCore/qnumeric.h>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {

struct ObjectCtor : FunctionObject {
    ObjectCtor(QV4::ExecutionContext *scope);
};

}

struct ObjectCtor: FunctionObject
{
    V4_OBJECT2(ObjectCtor, FunctionObject)

    static ReturnedValue construct(Managed *that, CallData *callData);
    static ReturnedValue call(Managed *that, CallData *callData);
};

struct ObjectPrototype: Object
{
    void init(ExecutionEngine *engine, Object *ctor);

    static ReturnedValue method_getPrototypeOf(CallContext *ctx);
    static ReturnedValue method_getOwnPropertyDescriptor(CallContext *ctx);
    static ReturnedValue method_getOwnPropertyNames(CallContext *context);
    static ReturnedValue method_create(CallContext *ctx);
    static ReturnedValue method_defineProperty(CallContext *ctx);
    static ReturnedValue method_defineProperties(CallContext *ctx);
    static ReturnedValue method_seal(CallContext *ctx);
    static ReturnedValue method_freeze(CallContext *ctx);
    static ReturnedValue method_preventExtensions(CallContext *ctx);
    static ReturnedValue method_isSealed(CallContext *ctx);
    static ReturnedValue method_isFrozen(CallContext *ctx);
    static ReturnedValue method_isExtensible(CallContext *ctx);
    static ReturnedValue method_keys(CallContext *ctx);

    static ReturnedValue method_toString(CallContext *ctx);
    static ReturnedValue method_toLocaleString(CallContext *ctx);
    static ReturnedValue method_valueOf(CallContext *ctx);
    static ReturnedValue method_hasOwnProperty(CallContext *ctx);
    static ReturnedValue method_isPrototypeOf(CallContext *ctx);
    static ReturnedValue method_propertyIsEnumerable(CallContext *ctx);

    static ReturnedValue method_defineGetter(CallContext *ctx);
    static ReturnedValue method_defineSetter(CallContext *ctx);

    static ReturnedValue method_get_proto(CallContext *ctx);
    static ReturnedValue method_set_proto(CallContext *ctx);

    static void toPropertyDescriptor(ExecutionEngine *engine, const Value &v, Property *desc, PropertyAttributes *attrs);
    static ReturnedValue fromPropertyDescriptor(ExecutionEngine *engine, const Property *desc, PropertyAttributes attrs);

    static Heap::ArrayObject *getOwnPropertyNames(ExecutionEngine *v4, const Value &o);
};


}

QT_END_NAMESPACE

#endif // QV4ECMAOBJECTS_P_H
