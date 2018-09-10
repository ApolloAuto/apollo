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
#ifndef QV4JSONOBJECT_H
#define QV4JSONOBJECT_H

#include "qv4object_p.h"
#include <qjsonarray.h>
#include <qjsonobject.h>
#include <qjsonvalue.h>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {

struct JsonObject : Object {
    JsonObject(ExecutionEngine *e);
};

}

struct JsonObject : Object {
    Q_MANAGED_TYPE(JsonObject)
    V4_OBJECT2(JsonObject, Object)
private:
    // ### GC
    typedef QSet<QV4::Heap::Base *> V4ObjectSet;
public:

    static ReturnedValue method_parse(CallContext *ctx);
    static ReturnedValue method_stringify(CallContext *ctx);

    static ReturnedValue fromJsonValue(ExecutionEngine *engine, const QJsonValue &value);
    static ReturnedValue fromJsonObject(ExecutionEngine *engine, const QJsonObject &object);
    static ReturnedValue fromJsonArray(ExecutionEngine *engine, const QJsonArray &array);

    static inline QJsonValue toJsonValue(const QV4::Value &value)
    { V4ObjectSet visitedObjects; return toJsonValue(value, visitedObjects); }
    static inline QJsonObject toJsonObject(QV4::Object *o)
    { V4ObjectSet visitedObjects; return toJsonObject(o, visitedObjects); }
    static inline QJsonArray toJsonArray(QV4::ArrayObject *a)
    { V4ObjectSet visitedObjects; return toJsonArray(a, visitedObjects); }

private:
    static QJsonValue toJsonValue(const QV4::Value &value, V4ObjectSet &visitedObjects);
    static QJsonObject toJsonObject(Object *o, V4ObjectSet &visitedObjects);
    static QJsonArray toJsonArray(ArrayObject *a, V4ObjectSet &visitedObjects);

};

}

QT_END_NAMESPACE

#endif

