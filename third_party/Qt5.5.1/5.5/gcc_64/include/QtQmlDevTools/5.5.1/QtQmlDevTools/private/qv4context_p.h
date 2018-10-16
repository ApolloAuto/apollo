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
#ifndef QMLJS_ENVIRONMENT_H
#define QMLJS_ENVIRONMENT_H

#include "qv4global_p.h"
#include "qv4managed_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace CompiledData {
struct CompilationUnit;
struct Function;
}

struct CallContext;
struct CatchContext;
struct WithContext;

struct CallData
{
    // below is to be compatible with Value. Initialize tag to 0
#if Q_BYTE_ORDER != Q_LITTLE_ENDIAN
    uint tag;
#endif
    int argc;
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
    uint tag;
#endif
    inline ReturnedValue argument(int i) const {
        return i < argc ? args[i].asReturnedValue() : Primitive::undefinedValue().asReturnedValue();
    }

    Value thisObject;
    Value args[1];
};

namespace Heap {

struct ExecutionContext : Base {
    enum ContextType {
        Type_GlobalContext = 0x1,
        Type_CatchContext = 0x2,
        Type_WithContext = 0x3,
        Type_SimpleCallContext = 0x4,
        Type_CallContext = 0x5,
        Type_QmlContext = 0x6
    };

    inline ExecutionContext(ExecutionEngine *engine, ContextType t);

    CallData *callData;

    ExecutionEngine *engine;
    ExecutionContext *parent;
    ExecutionContext *outer;
    Lookup *lookups;
    CompiledData::CompilationUnit *compilationUnit;

    ContextType type : 8;
    bool strictMode : 8;
    int lineNumber;
};

struct CallContext : ExecutionContext {
    CallContext(ExecutionEngine *engine, ContextType t = Type_SimpleCallContext)
        : ExecutionContext(engine, t)
    {
        function = 0;
        locals = 0;
        activation = 0;
    }
    CallContext(ExecutionEngine *engine, QV4::Object *qml, QV4::FunctionObject *function);

    FunctionObject *function;
    Value *locals;
    Object *activation;
};

struct GlobalContext : ExecutionContext {
    GlobalContext(ExecutionEngine *engine);
    Object *global;
};

struct CatchContext : ExecutionContext {
    CatchContext(ExecutionEngine *engine, QV4::String *exceptionVarName, const Value &exceptionValue);
    StringValue exceptionVarName;
    Value exceptionValue;
};

struct WithContext : ExecutionContext {
    WithContext(ExecutionEngine *engine, QV4::Object *with);
    Object *withObject;
};


}

struct Q_QML_EXPORT ExecutionContext : public Managed
{
    enum {
        IsExecutionContext = true
    };

    V4_MANAGED(ExecutionContext, Managed)
    Q_MANAGED_TYPE(ExecutionContext)

    ExecutionEngine *engine() const { return d()->engine; }

    Heap::CallContext *newCallContext(FunctionObject *f, CallData *callData);
    Heap::WithContext *newWithContext(Object *with);
    Heap::CatchContext *newCatchContext(String *exceptionVarName, const Value &exceptionValue);
    Heap::CallContext *newQmlContext(FunctionObject *f, Object *qml);

    void createMutableBinding(String *name, bool deletable);

    void setProperty(String *name, const Value &value);
    ReturnedValue getProperty(String *name);
    ReturnedValue getPropertyAndBase(String *name, Heap::Object **base);
    bool deleteProperty(String *name);

    inline CallContext *asCallContext();
    inline const CallContext *asCallContext() const;
    inline const CatchContext *asCatchContext() const;
    inline const WithContext *asWithContext() const;

    Heap::FunctionObject *getFunctionObject() const;

    static void markObjects(Heap::Base *m, ExecutionEngine *e);

    const Value &thisObject() const {
        return d()->callData->thisObject;
    }
    int argc() const {
        return d()->callData->argc;
    }
    const Value *args() const {
        return d()->callData->args;
    }
    ReturnedValue argument(int i) const {
        return d()->callData->argument(i);
    }
};

struct CallContext : public ExecutionContext
{
    V4_MANAGED(CallContext, ExecutionContext)

    // formals are in reverse order
    Identifier * const *formals() const;
    unsigned int formalCount() const;
    Identifier * const *variables() const;
    unsigned int variableCount() const;

    inline ReturnedValue argument(int i);
    bool needsOwnArguments() const;
};

inline ReturnedValue CallContext::argument(int i) {
    return i < argc() ? args()[i].asReturnedValue() : Primitive::undefinedValue().asReturnedValue();
}

struct GlobalContext : public ExecutionContext
{
    V4_MANAGED(GlobalContext, ExecutionContext)

};

struct CatchContext : public ExecutionContext
{
    V4_MANAGED(CatchContext, ExecutionContext)
};

struct WithContext : public ExecutionContext
{
    V4_MANAGED(WithContext, ExecutionContext)
};

inline CallContext *ExecutionContext::asCallContext()
{
    return d()->type >= Heap::ExecutionContext::Type_SimpleCallContext ? static_cast<CallContext *>(this) : 0;
}

inline const CallContext *ExecutionContext::asCallContext() const
{
    return d()->type >= Heap::ExecutionContext::Type_SimpleCallContext ? static_cast<const CallContext *>(this) : 0;
}

inline const CatchContext *ExecutionContext::asCatchContext() const
{
    return d()->type == Heap::ExecutionContext::Type_CatchContext ? static_cast<const CatchContext *>(this) : 0;
}

inline const WithContext *ExecutionContext::asWithContext() const
{
    return d()->type == Heap::ExecutionContext::Type_WithContext ? static_cast<const WithContext *>(this) : 0;
}

/* Function *f, int argc */
#define requiredMemoryForExecutionContect(f, argc) \
    ((sizeof(CallContext::Data) + 7) & ~7) + sizeof(Value) * (f->varCount() + qMax((uint)argc, f->formalParameterCount())) + sizeof(CallData)

} // namespace QV4

QT_END_NAMESPACE

#endif
