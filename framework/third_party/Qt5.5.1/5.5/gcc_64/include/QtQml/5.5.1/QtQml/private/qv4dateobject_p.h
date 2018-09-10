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
#ifndef QV4DATEOBJECT_P_H
#define QV4DATEOBJECT_P_H

#include "qv4object_p.h"
#include "qv4functionobject_p.h"
#include <QtCore/qnumeric.h>

QT_BEGIN_NAMESPACE

class QDateTime;

namespace QV4 {

namespace Heap {

struct DateObject : Object {
    DateObject(InternalClass *ic, QV4::Object *prototype)
        : Object(ic, prototype)
    {
        value = Encode(qSNaN());
    }

    DateObject(QV4::ExecutionEngine *engine, const Value &date)
        : Object(engine->emptyClass, engine->datePrototype.asObject())
    {
        value = date;
    }
    DateObject(QV4::ExecutionEngine *engine, const QDateTime &date);
    Value value;
};

struct DateCtor : FunctionObject {
    DateCtor(QV4::ExecutionContext *scope);
};

}

struct DateObject: Object {
    V4_OBJECT2(DateObject, Object)
    Q_MANAGED_TYPE(DateObject)


    Value date() const { return d()->value; }
    Value &date() { return d()->value; }
    void setDate(const Value &date) { d()->value = date; }

    QDateTime toQDateTime() const;
};

struct DateCtor: FunctionObject
{
    V4_OBJECT2(DateCtor, FunctionObject)

    static ReturnedValue construct(Managed *, CallData *callData);
    static ReturnedValue call(Managed *that, CallData *);
};

struct DatePrototype: DateObject
{
    void init(ExecutionEngine *engine, Object *ctor);

    static double getThisDate(ExecutionContext *ctx);

    static ReturnedValue method_parse(CallContext *ctx);
    static ReturnedValue method_UTC(CallContext *ctx);
    static ReturnedValue method_now(CallContext *ctx);

    static ReturnedValue method_toString(CallContext *ctx);
    static ReturnedValue method_toDateString(CallContext *ctx);
    static ReturnedValue method_toTimeString(CallContext *ctx);
    static ReturnedValue method_toLocaleString(CallContext *ctx);
    static ReturnedValue method_toLocaleDateString(CallContext *ctx);
    static ReturnedValue method_toLocaleTimeString(CallContext *ctx);
    static ReturnedValue method_valueOf(CallContext *ctx);
    static ReturnedValue method_getTime(CallContext *ctx);
    static ReturnedValue method_getYear(CallContext *ctx);
    static ReturnedValue method_getFullYear(CallContext *ctx);
    static ReturnedValue method_getUTCFullYear(CallContext *ctx);
    static ReturnedValue method_getMonth(CallContext *ctx);
    static ReturnedValue method_getUTCMonth(CallContext *ctx);
    static ReturnedValue method_getDate(CallContext *ctx);
    static ReturnedValue method_getUTCDate(CallContext *ctx);
    static ReturnedValue method_getDay(CallContext *ctx);
    static ReturnedValue method_getUTCDay(CallContext *ctx);
    static ReturnedValue method_getHours(CallContext *ctx);
    static ReturnedValue method_getUTCHours(CallContext *ctx);
    static ReturnedValue method_getMinutes(CallContext *ctx);
    static ReturnedValue method_getUTCMinutes(CallContext *ctx);
    static ReturnedValue method_getSeconds(CallContext *ctx);
    static ReturnedValue method_getUTCSeconds(CallContext *ctx);
    static ReturnedValue method_getMilliseconds(CallContext *ctx);
    static ReturnedValue method_getUTCMilliseconds(CallContext *ctx);
    static ReturnedValue method_getTimezoneOffset(CallContext *ctx);
    static ReturnedValue method_setTime(CallContext *ctx);
    static ReturnedValue method_setMilliseconds(CallContext *ctx);
    static ReturnedValue method_setUTCMilliseconds(CallContext *ctx);
    static ReturnedValue method_setSeconds(CallContext *ctx);
    static ReturnedValue method_setUTCSeconds(CallContext *ctx);
    static ReturnedValue method_setMinutes(CallContext *ctx);
    static ReturnedValue method_setUTCMinutes(CallContext *ctx);
    static ReturnedValue method_setHours(CallContext *ctx);
    static ReturnedValue method_setUTCHours(CallContext *ctx);
    static ReturnedValue method_setDate(CallContext *ctx);
    static ReturnedValue method_setUTCDate(CallContext *ctx);
    static ReturnedValue method_setMonth(CallContext *ctx);
    static ReturnedValue method_setUTCMonth(CallContext *ctx);
    static ReturnedValue method_setYear(CallContext *ctx);
    static ReturnedValue method_setFullYear(CallContext *ctx);
    static ReturnedValue method_setUTCFullYear(CallContext *ctx);
    static ReturnedValue method_toUTCString(CallContext *ctx);
    static ReturnedValue method_toISOString(CallContext *ctx);
    static ReturnedValue method_toJSON(CallContext *ctx);

    static void timezoneUpdated();
};

}

QT_END_NAMESPACE

#endif // QV4ECMAOBJECTS_P_H
