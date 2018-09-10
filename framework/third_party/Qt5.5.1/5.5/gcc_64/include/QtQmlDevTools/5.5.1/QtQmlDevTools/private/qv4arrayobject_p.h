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
#ifndef QV4ARRAYOBJECT_H
#define QV4ARRAYOBJECT_H

#include "qv4object_p.h"
#include "qv4functionobject_p.h"
#include <QtCore/qnumeric.h>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {

struct ArrayCtor : FunctionObject {
    ArrayCtor(QV4::ExecutionContext *scope);
};

}

struct ArrayCtor: FunctionObject
{
    V4_OBJECT2(ArrayCtor, FunctionObject)

    static ReturnedValue construct(Managed *m, CallData *callData);
    static ReturnedValue call(Managed *that, CallData *callData);
};

struct ArrayPrototype: ArrayObject
{
    void init(ExecutionEngine *engine, Object *ctor);

    static ReturnedValue method_isArray(CallContext *ctx);
    static ReturnedValue method_toString(CallContext *ctx);
    static ReturnedValue method_toLocaleString(CallContext *ctx);
    static ReturnedValue method_concat(CallContext *ctx);
    static ReturnedValue method_join(CallContext *ctx);
    static ReturnedValue method_pop(CallContext *ctx);
    static ReturnedValue method_push(CallContext *ctx);
    static ReturnedValue method_reverse(CallContext *ctx);
    static ReturnedValue method_shift(CallContext *ctx);
    static ReturnedValue method_slice(CallContext *ctx);
    static ReturnedValue method_sort(CallContext *ctx);
    static ReturnedValue method_splice(CallContext *ctx);
    static ReturnedValue method_unshift(CallContext *ctx);
    static ReturnedValue method_indexOf(CallContext *ctx);
    static ReturnedValue method_lastIndexOf(CallContext *ctx);
    static ReturnedValue method_every(CallContext *ctx);
    static ReturnedValue method_some(CallContext *ctx);
    static ReturnedValue method_forEach(CallContext *ctx);
    static ReturnedValue method_map(CallContext *ctx);
    static ReturnedValue method_filter(CallContext *ctx);
    static ReturnedValue method_reduce(CallContext *ctx);
    static ReturnedValue method_reduceRight(CallContext *ctx);
};


} // namespace QV4

QT_END_NAMESPACE

#endif // QV4ECMAOBJECTS_P_H
