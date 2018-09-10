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

#ifndef QQMLVALUETYPEWRAPPER_P_H
#define QQMLVALUETYPEWRAPPER_P_H

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

QT_BEGIN_NAMESPACE

class QV8Engine;
class QQmlValueType;

namespace QV4 {

namespace Heap {

struct QQmlValueTypeWrapper : Object {
    QQmlValueTypeWrapper(ExecutionEngine *engine);
    ~QQmlValueTypeWrapper();
    QQmlRefPointer<QQmlPropertyCache> propertyCache;
    mutable void *gadgetPtr;
    QQmlValueType *valueType;

    void setValue(const QVariant &value) const;
    QVariant toVariant() const;
};

}

struct Q_QML_EXPORT QQmlValueTypeWrapper : Object
{
    V4_OBJECT2(QQmlValueTypeWrapper, Object)
    static void destroy(Heap::Base *b);

public:

    static ReturnedValue create(ExecutionEngine *engine, QObject *, int, const QMetaObject *metaObject, int typeId);
    static ReturnedValue create(ExecutionEngine *engine, const QVariant &, const QMetaObject *metaObject, int typeId);

    QVariant toVariant() const;
    bool toGadget(void *data) const;
    bool isEqual(const QVariant& value);
    int typeId() const;
    bool write(QObject *target, int propertyIndex) const;

    static ReturnedValue get(Managed *m, String *name, bool *hasProperty);
    static void put(Managed *m, String *name, const Value &value);
    static bool isEqualTo(Managed *m, Managed *other);
    static PropertyAttributes query(const Managed *, String *name);

    static QV4::ReturnedValue method_toString(CallContext *ctx);

    static void initProto(ExecutionEngine *v4);
};

}

QT_END_NAMESPACE

#endif // QV8VALUETYPEWRAPPER_P_H


