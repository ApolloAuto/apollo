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
#ifndef QV4OBJECTITERATOR_H
#define QV4OBJECTITERATOR_H

#include "qv4global_p.h"
#include "qv4object_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct Q_QML_EXPORT ObjectIterator
{
    enum Flags {
        NoFlags = 0,
        EnumerableOnly = 0x1,
        WithProtoChain = 0x2,
    };

    ExecutionEngine *engine;
    Value *object;
    Value *current;
    SparseArrayNode *arrayNode;
    uint arrayIndex;
    uint memberIndex;
    uint flags;

    ObjectIterator(ExecutionEngine *e, Value *scratch1, Value *scratch2, Object *o, uint flags);
    ObjectIterator(Scope &scope, Object *o, uint flags);
    void init(Object *o);
    void next(Heap::String **name, uint *index, Property *pd, PropertyAttributes *attributes = 0);
    ReturnedValue nextPropertyName(Value *value);
    ReturnedValue nextPropertyNameAsString(Value *value);
    ReturnedValue nextPropertyNameAsString();
};

namespace Heap {
struct ForEachIteratorObject : Object {
    ForEachIteratorObject(QV4::ExecutionEngine *engine, QV4::Object *o);
    ObjectIterator it;
    Value workArea[2];
};

}

struct ForEachIteratorObject: Object {
    V4_OBJECT2(ForEachIteratorObject, Object)
    Q_MANAGED_TYPE(ForeachIteratorObject)

    ReturnedValue nextPropertyName() { return d()->it.nextPropertyNameAsString(); }

protected:
    static void markObjects(Heap::Base *that, ExecutionEngine *e);
};

inline
Heap::ForEachIteratorObject::ForEachIteratorObject(QV4::ExecutionEngine *engine, QV4::Object *o)
    : Heap::Object(engine)
    , it(engine, workArea, workArea + 1, o, ObjectIterator::EnumerableOnly|ObjectIterator::WithProtoChain)
{
}


}

QT_END_NAMESPACE

#endif
