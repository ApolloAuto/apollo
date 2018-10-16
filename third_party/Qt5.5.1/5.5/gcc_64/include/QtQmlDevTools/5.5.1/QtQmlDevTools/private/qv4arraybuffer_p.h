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
#ifndef QV4ARRAYBUFFER_H
#define QV4ARRAYBUFFER_H

#include "qv4object_p.h"
#include "qv4functionobject_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {

struct ArrayBufferCtor : FunctionObject {
    ArrayBufferCtor(QV4::ExecutionContext *scope);
};

struct Q_QML_PRIVATE_EXPORT ArrayBuffer : Object {
    ArrayBuffer(ExecutionEngine *e, size_t length);
    ArrayBuffer(ExecutionEngine *e, const QByteArray& array);
    ~ArrayBuffer();
    QTypedArrayData<char> *data;

    uint byteLength() const { return data->size; }
};

}

struct ArrayBufferCtor: FunctionObject
{
    V4_OBJECT2(ArrayBufferCtor, FunctionObject)

    static ReturnedValue construct(Managed *m, CallData *callData);
    static ReturnedValue call(Managed *that, CallData *callData);

    static ReturnedValue method_isView(CallContext *ctx);

};

struct Q_QML_PRIVATE_EXPORT ArrayBuffer : Object
{
    V4_OBJECT2(ArrayBuffer, Object)
    V4_NEEDS_DESTROY

    QByteArray asByteArray() const;
    uint byteLength() const { return d()->byteLength(); }
    char *data() { detach(); return d()->data ? d()->data->data() : 0; }
    const char *constData() { detach(); return d()->data ? d()->data->data() : 0; }

private:
    void detach();
};

struct ArrayBufferPrototype: Object
{
    void init(ExecutionEngine *engine, Object *ctor);

    static ReturnedValue method_get_byteLength(CallContext *ctx);
    static ReturnedValue method_slice(CallContext *ctx);
};


} // namespace QV4

QT_END_NAMESPACE

#endif
