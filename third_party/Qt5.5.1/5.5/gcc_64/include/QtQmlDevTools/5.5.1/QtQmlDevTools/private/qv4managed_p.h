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
#ifndef QMLJS_MANAGED_H
#define QMLJS_MANAGED_H

#include "qv4global_p.h"
#include "qv4value_p.h"
#include "qv4internalclass_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

#define Q_MANAGED_CHECK \
    template <typename _T> inline void qt_check_for_QMANAGED_macro(const _T *_q_argument) const \
    { int i = qYouForgotTheQ_MANAGED_Macro(this, _q_argument); i = i + 1; }

template <typename T>
inline int qYouForgotTheQ_MANAGED_Macro(T, T) { return 0; }

template <typename T1, typename T2>
inline void qYouForgotTheQ_MANAGED_Macro(T1, T2) {}

#ifdef Q_COMPILER_STATIC_ASSERT
#define V4_MANAGED_SIZE_TEST void __dataTest() { Q_STATIC_ASSERT(sizeof(*this) == sizeof(Managed)); }
#else
#define V4_MANAGED_SIZE_TEST
#endif

#define V4_NEEDS_DESTROY static void destroy(QV4::Heap::Base *b) { static_cast<Data *>(b)->~Data(); }


#define V4_MANAGED_ITSELF(DataClass, superClass) \
    public: \
        Q_MANAGED_CHECK \
        typedef QV4::Heap::DataClass Data; \
        typedef superClass SuperClass; \
        static const QV4::ManagedVTable static_vtbl; \
        static inline const QV4::ManagedVTable *staticVTable() { return &static_vtbl; } \
        V4_MANAGED_SIZE_TEST \
        QV4::Heap::DataClass *d() const { return static_cast<QV4::Heap::DataClass *>(m); }

#define V4_MANAGED(DataClass, superClass) \
    private: \
        DataClass() Q_DECL_EQ_DELETE; \
        Q_DISABLE_COPY(DataClass) \
        V4_MANAGED_ITSELF(DataClass, superClass)

#define Q_MANAGED_TYPE(type) \
    public: \
        enum { MyType = Type_##type };

#define Q_VTABLE_FUNCTION(classname, func) \
    (classname::func == QV4::Managed::func ? 0 : classname::func)


struct GCDeletable
{
    GCDeletable() : next(0), lastCall(false) {}
    virtual ~GCDeletable() {}
    GCDeletable *next;
    bool lastCall;
};

struct ManagedVTable
{
    const ManagedVTable * const parent;
    uint isExecutionContext : 1;
    uint isString : 1;
    uint isObject : 1;
    uint isFunctionObject : 1;
    uint isErrorObject : 1;
    uint isArrayData : 1;
    uint unused : 18;
    uint type : 8;
    const char *className;
    void (*destroy)(Heap::Base *);
    void (*markObjects)(Heap::Base *, ExecutionEngine *e);
    bool (*isEqualTo)(Managed *m, Managed *other);
};

#define DEFINE_MANAGED_VTABLE_INT(classname, parentVTable) \
{     \
    parentVTable, \
    classname::IsExecutionContext,   \
    classname::IsString,   \
    classname::IsObject,   \
    classname::IsFunctionObject,   \
    classname::IsErrorObject,   \
    classname::IsArrayData,   \
    0,                                          \
    classname::MyType,                          \
    #classname, \
    Q_VTABLE_FUNCTION(classname, destroy),                                    \
    markObjects,                                \
    isEqualTo                                  \
}

#define DEFINE_MANAGED_VTABLE(classname) \
const QV4::ManagedVTable classname::static_vtbl = DEFINE_MANAGED_VTABLE_INT(classname, 0)

struct Q_QML_PRIVATE_EXPORT Managed : Value
{
    V4_MANAGED_ITSELF(Base, Managed)
    enum {
        IsExecutionContext = false,
        IsString = false,
        IsObject = false,
        IsFunctionObject = false,
        IsErrorObject = false,
        IsArrayData = false
    };
private:
    void *operator new(size_t);
    Managed() Q_DECL_EQ_DELETE;
    Q_DISABLE_COPY(Managed)

public:

    inline void mark(QV4::ExecutionEngine *engine);

    enum Type {
        Type_Invalid,
        Type_String,
        Type_Object,
        Type_ArrayObject,
        Type_FunctionObject,
        Type_BooleanObject,
        Type_NumberObject,
        Type_StringObject,
        Type_DateObject,
        Type_RegExpObject,
        Type_ErrorObject,
        Type_ArgumentsObject,
        Type_JsonObject,
        Type_MathObject,

        Type_ExecutionContext,
        Type_ForeachIteratorObject,
        Type_RegExp,

        Type_QmlSequence
    };
    Q_MANAGED_TYPE(Invalid)

    template <typename T>
    T *as() {
        Q_ASSERT(d()->vtable);
#if !defined(QT_NO_QOBJECT_CHECK)
        static_cast<T *>(this)->qt_check_for_QMANAGED_macro(static_cast<T *>(this));
#endif
        const ManagedVTable *vt = d()->vtable;
        while (vt) {
            if (vt == T::staticVTable())
                return static_cast<T *>(this);
            vt = vt->parent;
        }
        return 0;
    }
    template <typename T>
    const T *as() const {
        Q_ASSERT(d()->vtable);
#if !defined(QT_NO_QOBJECT_CHECK)
        static_cast<T *>(this)->qt_check_for_QMANAGED_macro(static_cast<T *>(const_cast<Managed *>(this)));
#endif
        const ManagedVTable *vt = d()->vtable;
        while (vt) {
            if (vt == T::staticVTable())
                return static_cast<T *>(this);
            vt = vt->parent;
        }
        return 0;
    }

    String *asString() { return d()->vtable->isString ? reinterpret_cast<String *>(this) : 0; }
    Object *asObject() { return d()->vtable->isObject ? reinterpret_cast<Object *>(this) : 0; }
    ArrayObject *asArrayObject() { return d()->vtable->type == Type_ArrayObject ? reinterpret_cast<ArrayObject *>(this) : 0; }
    FunctionObject *asFunctionObject() { return d()->vtable->isFunctionObject ? reinterpret_cast<FunctionObject *>(this) : 0; }
    BooleanObject *asBooleanObject() { return d()->vtable->type == Type_BooleanObject ? reinterpret_cast<BooleanObject *>(this) : 0; }
    NumberObject *asNumberObject() { return d()->vtable->type == Type_NumberObject ? reinterpret_cast<NumberObject *>(this) : 0; }
    StringObject *asStringObject() { return d()->vtable->type == Type_StringObject ? reinterpret_cast<StringObject *>(this) : 0; }
    DateObject *asDateObject() { return d()->vtable->type == Type_DateObject ? reinterpret_cast<DateObject *>(this) : 0; }
    ErrorObject *asErrorObject() { return d()->vtable->isErrorObject ? reinterpret_cast<ErrorObject *>(this) : 0; }
    ArgumentsObject *asArgumentsObject() { return d()->vtable->type == Type_ArgumentsObject ? reinterpret_cast<ArgumentsObject *>(this) : 0; }

    bool isListType() const { return d()->vtable->type == Type_QmlSequence; }

    bool isArrayObject() const { return d()->vtable->type == Type_ArrayObject; }
    bool isStringObject() const { return d()->vtable->type == Type_StringObject; }

    QString className() const;

    bool isEqualTo(const Managed *other) const
    { return d()->vtable->isEqualTo(const_cast<Managed *>(this), const_cast<Managed *>(other)); }

    static bool isEqualTo(Managed *m, Managed *other);

    bool inUse() const { return d()->inUse(); }
    bool markBit() const { return d()->isMarked(); }

    static void destroy(Heap::Base *) {}
private:
    friend class MemoryManager;
    friend struct Identifiers;
    friend struct ObjectIterator;
};


template<>
inline Managed *value_cast(const Value &v) {
    return v.asManaged();
}

template<typename T>
inline T *managed_cast(Managed *m)
{
    return m ? m->as<T>() : 0;
}

template<>
inline String *managed_cast(Managed *m)
{
    return m ? m->asString() : 0;
}
template<>
inline Object *managed_cast(Managed *m)
{
    return m ? m->asObject() : 0;
}
template<>
inline FunctionObject *managed_cast(Managed *m)
{
    return m ? m->asFunctionObject() : 0;
}

inline Value Value::fromManaged(Managed *m)
{
    if (!m)
        return QV4::Primitive::undefinedValue();
    return *m;
}

}


QT_END_NAMESPACE

#endif
