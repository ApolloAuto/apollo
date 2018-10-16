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
#ifndef QV4VALUE_P_H
#define QV4VALUE_P_H

#include <limits.h>

#include <QtCore/QString>
#include "qv4global_p.h"

/* We cannot rely on QT_POINTER_SIZE to be set correctly on host builds. In qmldevtools the Value objects
   are only used to store primitives, never object pointers. So we can use the 64-bit encoding. */
#ifdef V4_BOOTSTRAP
#define QV4_USE_64_BIT_VALUE_ENCODING
#elif QT_POINTER_SIZE == 8
#define QV4_USE_64_BIT_VALUE_ENCODING
#endif

QT_BEGIN_NAMESPACE

namespace QV4 {

typedef uint Bool;

namespace Heap {

struct Q_QML_EXPORT Base {
    union {
        const ManagedVTable *vtable;
        quintptr mm_data;
    };

    inline ReturnedValue asReturnedValue() const;
    inline void mark(QV4::ExecutionEngine *engine);

    enum {
        MarkBit = 0x1,
        NotInUse = 0x2,
        PointerMask = ~0x3
    };

    ManagedVTable *gcGetVtable() const {
        return reinterpret_cast<ManagedVTable *>(mm_data & PointerMask);
    }
    inline bool isMarked() const {
        return mm_data & MarkBit;
    }
    inline void setMarkBit() {
        mm_data |= MarkBit;
    }
    inline void clearMarkBit() {
        mm_data &= ~MarkBit;
    }

    inline bool inUse() const {
        return !(mm_data & NotInUse);
    }

    Base *nextFree() {
        return reinterpret_cast<Base *>(mm_data & PointerMask);
    }
    void setNextFree(Base *m) {
        mm_data = (reinterpret_cast<quintptr>(m) | NotInUse);
    }

    void *operator new(size_t, Managed *m) { return m; }
    void *operator new(size_t, Heap::Base *m) { return m; }
    void operator delete(void *, Heap::Base *) {}
};

}

struct Q_QML_PRIVATE_EXPORT Value
{
    /*
        We use two different ways of encoding JS values. One for 32bit and one for 64bit systems.

        In both cases, we use 8 bytes for a value and a different variant of NaN boxing. A Double NaN (actually -qNaN)
        is indicated by a number that has the top 13 bits set. The other values are usually set to 0 by the
        processor, and are thus free for us to store other data. We keep pointers in there for managed objects,
        and encode the other types using the free space given to use by the unused bits for NaN values. This also
        works for pointers on 64 bit systems, as they all currently only have 48 bits of addressable memory.

        On 32bit, we store doubles as doubles. All other values, have the high 32bits set to a value that
        will make the number a NaN. The Masks below are used for encoding the other types.

        On 64 bit, we xor Doubles with (0xffff8000 << 32). That has the effect that no doubles will get encoded
        with the 13 highest bits all 0. We are now using special values for bits 14-17 to encode our values. These
        can be used, as the highest valid pointer on a 64 bit system is 2^48-1.

        If they are all 0, we have a pointer to a Managed object. If bit 14 is set we have an integer.
        This makes testing for pointers and numbers very fast (we have a number if any of the highest 14 bits is set).

        Bit 15-17 is then used to encode other immediates.
    */


    union {
        quint64 val;
#ifdef QV4_USE_64_BIT_VALUE_ENCODING
        Heap::Base *m;
#else
        double dbl;
#endif
        struct {
#if Q_BYTE_ORDER != Q_LITTLE_ENDIAN
            uint tag;
#endif
            union {
                uint uint_32;
                int int_32;
#ifndef QV4_USE_64_BIT_VALUE_ENCODING
                Heap::Base *m;
#endif
            };
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
            uint tag;
#endif
        };
    };

#ifndef QV4_USE_64_BIT_VALUE_ENCODING
    enum Masks {
        SilentNaNBit           =                  0x00040000,
        NaN_Mask               =                  0x7ff80000,
        NotDouble_Mask         =                  0x7ffa0000,
        Type_Mask              =                  0xffffc000,
        Immediate_Mask         = NotDouble_Mask | 0x00004000 | SilentNaNBit,
        IsNullOrUndefined_Mask = Immediate_Mask |    0x08000,
        Tag_Shift = 32
    };
    enum ValueType {
        Undefined_Type = Immediate_Mask | 0x00000,
        Null_Type      = Immediate_Mask | 0x10000,
        Boolean_Type   = Immediate_Mask | 0x08000,
        Integer_Type   = Immediate_Mask | 0x18000,
        Managed_Type   = NotDouble_Mask | 0x00000 | SilentNaNBit,
        Empty_Type     = NotDouble_Mask | 0x18000 | SilentNaNBit
    };

    enum ImmediateFlags {
        ConvertibleToInt = Immediate_Mask | 0x1
    };

    enum ValueTypeInternal {
        _Null_Type = Null_Type | ConvertibleToInt,
        _Boolean_Type = Boolean_Type | ConvertibleToInt,
        _Integer_Type = Integer_Type | ConvertibleToInt,

    };
#else
    static const quint64 NaNEncodeMask = 0xffff800000000000ll;
    static const quint64 IsInt32Mask  = 0x0002000000000000ll;
    static const quint64 IsDoubleMask = 0xfffc000000000000ll;
    static const quint64 IsNumberMask = IsInt32Mask|IsDoubleMask;
    static const quint64 IsNullOrUndefinedMask = 0x0000800000000000ll;
    static const quint64 IsNullOrBooleanMask = 0x0001000000000000ll;
    static const quint64 IsConvertibleToIntMask = IsInt32Mask|IsNullOrBooleanMask;

    enum Masks {
        NaN_Mask = 0x7ff80000,
        Type_Mask = 0xffff8000,
        IsDouble_Mask = 0xfffc0000,
        Immediate_Mask = 0x00018000,
        IsNullOrUndefined_Mask = 0x00008000,
        IsNullOrBoolean_Mask = 0x00010000,
        Tag_Shift = 32
    };
    enum ValueType {
        Undefined_Type = IsNullOrUndefined_Mask,
        Null_Type = IsNullOrUndefined_Mask|IsNullOrBoolean_Mask,
        Boolean_Type = IsNullOrBoolean_Mask,
        Integer_Type = 0x20000|IsNullOrBoolean_Mask,
        Managed_Type = 0,
        Empty_Type = Undefined_Type | 0x4000
    };
    enum {
        IsDouble_Shift = 64-14,
        IsNumber_Shift = 64-15,
        IsConvertibleToInt_Shift = 64-16,
        IsManaged_Shift = 64-17
    };


    enum ValueTypeInternal {
        _Null_Type = Null_Type,
        _Boolean_Type = Boolean_Type,
        _Integer_Type = Integer_Type
    };
#endif

    inline unsigned type() const {
        return tag & Type_Mask;
    }

    // used internally in property
    inline bool isEmpty() const { return tag == Empty_Type; }

    inline bool isUndefined() const { return tag == Undefined_Type; }
    inline bool isNull() const { return tag == _Null_Type; }
    inline bool isBoolean() const { return tag == _Boolean_Type; }
#ifdef QV4_USE_64_BIT_VALUE_ENCODING
    inline bool isInteger() const { return (val >> IsNumber_Shift) == 1; }
    inline bool isDouble() const { return (val >> IsDouble_Shift); }
    inline bool isNumber() const { return (val >> IsNumber_Shift); }
    inline bool isManaged() const { return !(val >> IsManaged_Shift); }
    inline bool isNullOrUndefined() const { return ((val >> IsManaged_Shift) & ~2) == 1; }
    inline bool integerCompatible() const { return ((val >> IsConvertibleToInt_Shift) & ~2) == 1; }
    static inline bool integerCompatible(Value a, Value b) {
        return a.integerCompatible() && b.integerCompatible();
    }
    static inline bool bothDouble(Value a, Value b) {
        return a.isDouble() && b.isDouble();
    }
    double doubleValue() const {
        Q_ASSERT(isDouble());
        union {
            quint64 i;
            double d;
        } v;
        v.i = val ^ NaNEncodeMask;
        return v.d;
    }
    void setDouble(double d) {
        union {
            quint64 i;
            double d;
        } v;
        v.d = d;
        val = v.i ^ NaNEncodeMask;
        Q_ASSERT(isDouble());
    }
    inline bool isNaN() const { return (tag & 0x7fff8000) == 0x00078000; }
#else
    inline bool isInteger() const { return tag == _Integer_Type; }
    inline bool isDouble() const { return (tag & NotDouble_Mask) != NotDouble_Mask; }
    inline bool isNumber() const { return tag == _Integer_Type || (tag & NotDouble_Mask) != NotDouble_Mask; }
    inline bool isManaged() const { return tag == Managed_Type; }
    inline bool isNullOrUndefined() const { return (tag & IsNullOrUndefined_Mask) == Undefined_Type; }
    inline bool integerCompatible() const { return (tag & ConvertibleToInt) == ConvertibleToInt; }
    static inline bool integerCompatible(Value a, Value b) {
        return ((a.tag & b.tag) & ConvertibleToInt) == ConvertibleToInt;
    }
    static inline bool bothDouble(Value a, Value b) {
        return ((a.tag | b.tag) & NotDouble_Mask) != NotDouble_Mask;
    }
    double doubleValue() const { Q_ASSERT(isDouble()); return dbl; }
    void setDouble(double d) { dbl = d; Q_ASSERT(isDouble()); }
    inline bool isNaN() const { return (tag & QV4::Value::NotDouble_Mask) == QV4::Value::NaN_Mask; }
#endif
    inline bool isString() const;
    inline bool isObject() const;
    inline bool isInt32() {
        if (tag == _Integer_Type)
            return true;
        if (isDouble()) {
            double d = doubleValue();
            int i = (int)d;
            if (i == d) {
                int_32 = i;
                tag = _Integer_Type;
                return true;
            }
        }
        return false;
    }
    double asDouble() const {
        if (tag == _Integer_Type)
            return int_32;
        return doubleValue();
    }

    bool booleanValue() const {
        return int_32;
    }
    int integerValue() const {
        return int_32;
    }

    String *stringValue() const {
        return m ? reinterpret_cast<String*>(const_cast<Value *>(this)) : 0;
    }
    Object *objectValue() const {
        return m ? reinterpret_cast<Object*>(const_cast<Value *>(this)) : 0;
    }
    Managed *managed() const {
        return m ? reinterpret_cast<Managed*>(const_cast<Value *>(this)) : 0;
    }
    Heap::Base *heapObject() const {
        return m;
    }

    quint64 rawValue() const {
        return val;
    }

    static inline Value fromHeapObject(Heap::Base *m)
    {
        Value v;
        v.m = m;
#ifndef QV4_USE_64_BIT_VALUE_ENCODING
        v.tag = Managed_Type;
#endif
        return v;
    }

    static inline Value fromManaged(Managed *m);

    int toUInt16() const;
    inline int toInt32() const;
    inline unsigned int toUInt32() const;

    inline bool toBoolean() const;
    double toInteger() const;
    inline double toNumber() const;
    double toNumberImpl() const;
    QString toQStringNoThrow() const;
    QString toQString() const;
    Heap::String *toString(ExecutionEngine *e) const;
    Heap::Object *toObject(ExecutionEngine *e) const;

    inline bool isPrimitive() const;
    inline bool tryIntegerConversion() {
        bool b = integerCompatible();
        if (b)
            tag = _Integer_Type;
        return b;
    }

    inline String *asString() const;
    inline Managed *asManaged() const;
    inline Object *asObject() const;
    inline FunctionObject *asFunctionObject() const;
    inline NumberObject *asNumberObject() const;
    inline StringObject *asStringObject() const;
    inline DateObject *asDateObject() const;
    inline ArrayObject *asArrayObject() const;
    inline ErrorObject *asErrorObject() const;

    template<typename T> inline T *as() const;
    template<typename T> inline T *cast() {
        return static_cast<T *>(managed());
    }
    template<typename T> inline const T *cast() const {
        return static_cast<const T *>(managed());
    }

    inline uint asArrayIndex() const;
    inline uint asArrayLength(bool *ok) const;

    ReturnedValue asReturnedValue() const { return val; }
    static Value fromReturnedValue(ReturnedValue val) { Value v; v.val = val; return v; }

    // Section 9.12
    bool sameValue(Value other) const;

    inline void mark(ExecutionEngine *e) const;

    Value &operator =(const ScopedValue &v);
    Value &operator=(ReturnedValue v) { val = v; return *this; }
    Value &operator=(Managed *m) {
        val = Value::fromManaged(m).val;
        return *this;
    }
    Value &operator=(Heap::Base *o) {
        m = o;
#ifndef QV4_USE_64_BIT_VALUE_ENCODING
        tag = Managed_Type;
#endif
        return *this;
    }

    template<typename T>
    Value &operator=(const Scoped<T> &t);
    Value &operator=(const Value &v) {
        val = v.val;
        return *this;
    }
};

inline Managed *Value::asManaged() const
{
    if (isManaged())
        return managed();
    return 0;
}

struct Q_QML_PRIVATE_EXPORT Primitive : public Value
{
    inline static Primitive emptyValue();
    static inline Primitive fromBoolean(bool b);
    static inline Primitive fromInt32(int i);
    inline static Primitive undefinedValue();
    static inline Primitive nullValue();
    static inline Primitive fromDouble(double d);
    static inline Primitive fromUInt32(uint i);

    using Value::toInt32;
    using Value::toUInt32;

    static double toInteger(double fromNumber);
    static int toInt32(double value);
    static unsigned int toUInt32(double value);
};

inline Primitive Primitive::undefinedValue()
{
    Primitive v;
#ifdef QV4_USE_64_BIT_VALUE_ENCODING
    v.val = quint64(Undefined_Type) << Tag_Shift;
#else
    v.tag = Undefined_Type;
    v.int_32 = 0;
#endif
    return v;
}

inline Primitive Primitive::emptyValue()
{
    Primitive v;
    v.tag = Value::Empty_Type;
    v.uint_32 = 0;
    return v;
}

template <typename T>
struct TypedValue : public Value
{
    template<typename X>
    TypedValue &operator =(X *x) {
        m = x;
#ifndef QV4_USE_64_BIT_VALUE_ENCODING
        tag = Managed_Type;
#endif
        return *this;
    }
    TypedValue &operator =(T *t);
    TypedValue &operator =(const Scoped<T> &v);
//    TypedValue &operator =(const ManagedRef<T> &v);

    TypedValue &operator =(const TypedValue<T> &t);

    bool operator!() const { return !managed(); }

    operator T *() { return static_cast<T *>(managed()); }
    T *operator->() { return static_cast<T *>(managed()); }
    const T *operator->() const { return static_cast<T *>(managed()); }
    T *getPointer() const { return static_cast<T *>(managed()); }

    void mark(ExecutionEngine *e) { if (managed()) managed()->mark(e); }
};
typedef TypedValue<String> StringValue;


struct Encode {
    static ReturnedValue undefined() {
        return quint64(Value::Undefined_Type) << Value::Tag_Shift;
    }
    static ReturnedValue null() {
        return quint64(Value::_Null_Type) << Value::Tag_Shift;
    }

    Encode(bool b) {
        val = (quint64(Value::_Boolean_Type) << Value::Tag_Shift) | (uint)b;
    }
    Encode(double d) {
        Value v;
        v.setDouble(d);
        val = v.val;
    }
    Encode(int i) {
        val = (quint64(Value::_Integer_Type) << Value::Tag_Shift) | (uint)i;
    }
    Encode(uint i) {
        if (i <= INT_MAX) {
            val = (quint64(Value::_Integer_Type) << Value::Tag_Shift) | i;
        } else {
            Value v;
            v.setDouble(i);
            val = v.val;
        }
    }
    Encode(ReturnedValue v) {
        val = v;
    }

    Encode(Heap::Base *o) {
        Q_ASSERT(o);
        val = Value::fromHeapObject(o).asReturnedValue();
    }

    operator ReturnedValue() const {
        return val;
    }
    quint64 val;
private:
    Encode(void *);
};

inline
ReturnedValue Heap::Base::asReturnedValue() const
{
    return Value::fromHeapObject(const_cast<Heap::Base *>(this)).asReturnedValue();
}


template<typename T>
T *value_cast(const Value &v)
{
    return v.as<T>();
}

template<typename T>
ReturnedValue value_convert(ExecutionEngine *e, const Value &v);

}

QT_END_NAMESPACE

#endif // QV4VALUE_DEF_P_H
