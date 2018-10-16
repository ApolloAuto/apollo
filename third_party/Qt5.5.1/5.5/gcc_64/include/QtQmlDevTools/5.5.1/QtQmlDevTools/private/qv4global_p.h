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

#ifndef QV4GLOBAL_H
#define QV4GLOBAL_H

#if defined(QT_BUILD_QMLDEVTOOLS_LIB) || defined(QT_QMLDEVTOOLS_LIB)
#define V4_BOOTSTRAP
#endif

#include <QtCore/qglobal.h>
#include <QString>

#ifdef V4_BOOTSTRAP
#include <private/qtqmldevtoolsglobal_p.h>
#else
#include <qtqmlglobal.h>
#include <private/qtqmlglobal_p.h>
#endif

#if defined(Q_CC_MSVC)
#include <float.h>
#include <math.h>

namespace std {

inline bool isinf(double d) { return !_finite(d) && !_isnan(d); }
inline bool isnan(double d) { return !!_isnan(d); }
inline bool isfinite(double d) { return _finite(d); }
#if _MSC_VER < 1800
inline bool signbit(double d) { return _copysign(1.0, d) < 0; }
#endif

} // namespace std

inline double trunc(double d) { return d > 0 ? floor(d) : ceil(d); }
#endif

#define qOffsetOf(s, m) ((size_t)((((char *)&(((s *)64)->m)) - 64)))

// Decide whether to enable or disable the JIT

// White list architectures
//
// NOTE: This should match the logic in qv4targetplatform_p.h!

#if defined(Q_PROCESSOR_X86) && !defined(__ILP32__) \
    && (defined(Q_OS_WIN) || defined(Q_OS_LINUX) || defined(Q_OS_QNX) || defined(Q_OS_FREEBSD))
#define V4_ENABLE_JIT
#elif defined(Q_PROCESSOR_X86_64) && !defined(__ILP32__) \
    && (defined(Q_OS_WIN) || defined(Q_OS_LINUX) || defined(Q_OS_MAC) || defined(Q_OS_FREEBSD))
#define V4_ENABLE_JIT
#elif defined(Q_PROCESSOR_ARM_32)

#if defined(thumb2) || defined(__thumb2__) || ((defined(__thumb) || defined(__thumb__)) && __TARGET_ARCH_THUMB-0 == 4)
#define V4_ENABLE_JIT
#elif defined(__ARM_ARCH_ISA_THUMB) && __ARM_ARCH_ISA_THUMB == 2 // clang 3.5 and later will set this if the core supports the Thumb-2 ISA.
#define V4_ENABLE_JIT
#endif

#endif

// Black list some platforms
#if defined(V4_ENABLE_JIT)
#if defined(Q_OS_IOS) || defined(Q_OS_WINRT)
#    undef V4_ENABLE_JIT
#endif
#endif

// Do certain things depending on whether the JIT is enabled or disabled

#ifdef V4_ENABLE_JIT
#define ENABLE_YARR_JIT 1
#define ENABLE_JIT 1
#define ENABLE_ASSEMBLER 1
#else
#define ENABLE_YARR_JIT 0
#define ENABLE_ASSEMBLER 0
#define ENABLE_JIT 0
#endif

#if defined(Q_OS_QNX)
#include <math.h>
#undef isnan
#undef isfinite
#undef isinf
#undef signbit
#endif

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {
    struct Base;
    struct MemberData;
    struct ArrayData;

    struct String;
    struct Object;
    struct ObjectPrototype;

    struct ExecutionContext;
    struct GlobalContext;
    struct CallContext;
    struct ScriptFunction;

    struct BooleanObject;
    struct NumberObject;
    struct StringObject;
    struct ArrayObject;
    struct DateObject;
    struct FunctionObject;
    struct ErrorObject;
    struct ArgumentsObject;
    struct QObjectWrapper;
    struct RegExpObject;
    struct RegExp;
    struct EvalFunction;

    struct ArrayBuffer;
    struct DataView;
    struct TypedArray;

}

class MemoryManager;
struct String;
struct Object;
struct ObjectPrototype;
struct ObjectIterator;
struct ExecutionContext;
struct GlobalContext;
struct CallContext;
struct ScriptFunction;
struct InternalClass;
struct Property;
struct Value;
struct Lookup;
struct ArrayData;
struct ManagedVTable;

struct BooleanObject;
struct NumberObject;
struct StringObject;
struct ArrayObject;
struct DateObject;
struct FunctionObject;
struct ErrorObject;
struct ArgumentsObject;
struct Managed;
struct ExecutionEngine;
struct QObjectWrapper;
struct RegExpObject;
struct RegExp;
struct EvalFunction;

struct ArrayBuffer;
struct DataView;
struct TypedArray;

// ReturnedValue is used to return values from runtime methods
// the type has to be a primitive type (no struct or union), so that the compiler
// will return it in a register on all platforms.
// It will be returned in rax on x64, [eax,edx] on x86 and [r0,r1] on arm
typedef quint64 ReturnedValue;
struct CallData;
struct Scope;
struct ScopedValue;
template<typename T> struct Scoped;
typedef Scoped<String> ScopedString;
typedef Scoped<Object> ScopedObject;
typedef Scoped<ArrayObject> ScopedArrayObject;
typedef Scoped<FunctionObject> ScopedFunctionObject;
typedef Scoped<ExecutionContext> ScopedContext;

struct PersistentValueStorage;
class PersistentValue;
class WeakValue;

struct IdentifierTable;
class RegExpCache;
class MultiplyWrappedQObjectMap;
struct QmlExtensions;

namespace Global {
    enum {
        ReservedArgumentCount = 6
    };
}

enum PropertyFlag {
    Attr_Data = 0,
    Attr_Accessor = 0x1,
    Attr_NotWritable = 0x2,
    Attr_NotEnumerable = 0x4,
    Attr_NotConfigurable = 0x8,
    Attr_ReadOnly = Attr_NotWritable|Attr_NotEnumerable|Attr_NotConfigurable,
    Attr_Invalid = 0xff
};

Q_DECLARE_FLAGS(PropertyFlags, PropertyFlag)
Q_DECLARE_OPERATORS_FOR_FLAGS(PropertyFlags)

struct PropertyAttributes
{
    union {
        uchar m_all;
        struct {
            uchar m_flags : 4;
            uchar m_mask : 4;
        };
        struct {
            uchar m_type : 1;
            uchar m_writable : 1;
            uchar m_enumerable : 1;
            uchar m_configurable : 1;
            uchar type_set : 1;
            uchar writable_set : 1;
            uchar enumerable_set : 1;
            uchar configurable_set : 1;
        };
    };

    enum Type {
        Data = 0,
        Accessor = 1,
        Generic = 2
    };

    PropertyAttributes() : m_all(0) {}
    PropertyAttributes(PropertyFlag f) : m_all(0) {
        if (f != Attr_Invalid) {
            setType(f & Attr_Accessor ? Accessor : Data);
            if (!(f & Attr_Accessor))
                setWritable(!(f & Attr_NotWritable));
            setEnumerable(!(f & Attr_NotEnumerable));
            setConfigurable(!(f & Attr_NotConfigurable));
        }
    }
    PropertyAttributes(PropertyFlags f) : m_all(0) {
        if (f != Attr_Invalid) {
            setType(f & Attr_Accessor ? Accessor : Data);
            if (!(f & Attr_Accessor))
                setWritable(!(f & Attr_NotWritable));
            setEnumerable(!(f & Attr_NotEnumerable));
            setConfigurable(!(f & Attr_NotConfigurable));
        }
    }
    PropertyAttributes(const PropertyAttributes &other) : m_all(other.m_all) {}
    PropertyAttributes & operator=(const PropertyAttributes &other) { m_all = other.m_all; return *this; }

    void setType(Type t) { m_type = t; type_set = true; }
    Type type() const { return type_set ? (Type)m_type : Generic; }

    bool isData() const { return type() == PropertyAttributes::Data || writable_set; }
    bool isAccessor() const { return type() == PropertyAttributes::Accessor; }
    bool isGeneric() const { return type() == PropertyAttributes::Generic && !writable_set; }

    bool hasType() const { return type_set; }
    bool hasWritable() const { return writable_set; }
    bool hasConfigurable() const { return configurable_set; }
    bool hasEnumerable() const { return enumerable_set; }

    void setWritable(bool b) { m_writable = b; writable_set = true; }
    void setConfigurable(bool b) { m_configurable = b; configurable_set = true; }
    void setEnumerable(bool b) { m_enumerable = b; enumerable_set = true; }

    void resolve() { m_mask = 0xf; if (m_type == Accessor) { m_writable = false; writable_set = false; } }

    bool isWritable() const { return m_type != Data || m_writable; }
    bool isEnumerable() const { return m_enumerable; }
    bool isConfigurable() const { return m_configurable; }

    void clearType() { m_type = Data; type_set = false; }
    void clearWritable() { m_writable = false; writable_set = false; }
    void clearEnumerable() { m_enumerable = false; enumerable_set = false; }
    void clearConfigurable() { m_configurable = false; configurable_set = false; }

    void clear() { m_all = 0; }
    bool isEmpty() const { return !m_all; }

    uint flags() const { return m_flags; }

    bool operator==(PropertyAttributes other) {
        return m_all == other.m_all;
    }
    bool operator!=(PropertyAttributes other) {
        return m_all != other.m_all;
    }
};

struct StackFrame {
    QString source;
    QString function;
    int line;
    int column;
};
typedef QVector<StackFrame> StackTrace;

}

Q_DECLARE_TYPEINFO(QV4::PropertyAttributes, Q_PRIMITIVE_TYPE);

QT_END_NAMESPACE

#endif // QV4GLOBAL_H
