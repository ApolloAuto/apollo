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
#ifndef QV4PROPERTYDESCRIPTOR_H
#define QV4PROPERTYDESCRIPTOR_H

#include "qv4global_p.h"
#include "qv4value_p.h"
#include "qv4internalclass_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct FunctionObject;

struct Property {
    Value value;
    Value set;

    // Section 8.10
    inline void fullyPopulated(PropertyAttributes *attrs) {
        if (!attrs->hasType()) {
            value = Primitive::undefinedValue();
        }
        if (attrs->type() == PropertyAttributes::Accessor) {
            attrs->clearWritable();
            if (value.isEmpty())
                value = Primitive::undefinedValue();
            if (set.isEmpty())
                set = Primitive::undefinedValue();
        }
        attrs->resolve();
    }

    static Property genericDescriptor() {
        Property pd;
        pd.value = Primitive::emptyValue();
        return pd;
    }

    inline bool isSubset(const PropertyAttributes &attrs, const Property *other, PropertyAttributes otherAttrs) const;
    inline void merge(PropertyAttributes &attrs, const Property *other, PropertyAttributes otherAttrs);

    inline Heap::FunctionObject *getter() const { return value.isManaged() ? reinterpret_cast<Heap::FunctionObject *>(value.heapObject()) : 0; }
    inline Heap::FunctionObject *setter() const { return set.isManaged() ? reinterpret_cast<Heap::FunctionObject *>(set.heapObject()) : 0; }
    inline void setGetter(FunctionObject *g) { value = Primitive::fromManaged(reinterpret_cast<Managed *>(g)); }
    inline void setSetter(FunctionObject *s) { set = s ? Primitive::fromManaged(reinterpret_cast<Managed *>(s)) : Value::fromHeapObject(0); }

    void copy(const Property *other, PropertyAttributes attrs) {
        value = other->value;
        if (attrs.isAccessor())
            set = other->set;
    }

    explicit Property()  { value = Encode::undefined(); set = Value::fromHeapObject(0); }
    explicit Property(Value v) : value(v) { set = Value::fromHeapObject(0); }
    Property(FunctionObject *getter, FunctionObject *setter) {
        value = Primitive::fromManaged(reinterpret_cast<Managed *>(getter));
        set = Primitive::fromManaged(reinterpret_cast<Managed *>(setter));
    }
    Property(Heap::FunctionObject *getter, Heap::FunctionObject *setter) {
        value.m = reinterpret_cast<Heap::Base *>(getter);
        set.m = reinterpret_cast<Heap::Base *>(setter);
    }
    Property &operator=(Value v) { value = v; return *this; }
private:
    Property(const Property &);
    Property &operator=(const Property &);
};

inline bool Property::isSubset(const PropertyAttributes &attrs, const Property *other, PropertyAttributes otherAttrs) const
{
    if (attrs.type() != PropertyAttributes::Generic && attrs.type() != otherAttrs.type())
        return false;
    if (attrs.hasEnumerable() && attrs.isEnumerable() != otherAttrs.isEnumerable())
        return false;
    if (attrs.hasConfigurable() && attrs.isConfigurable() != otherAttrs.isConfigurable())
        return false;
    if (attrs.hasWritable() && attrs.isWritable() != otherAttrs.isWritable())
        return false;
    if (attrs.type() == PropertyAttributes::Data && !value.sameValue(other->value))
        return false;
    if (attrs.type() == PropertyAttributes::Accessor) {
        if (value.heapObject() != other->value.heapObject())
            return false;
        if (set.heapObject() != other->set.heapObject())
            return false;
    }
    return true;
}

inline void Property::merge(PropertyAttributes &attrs, const Property *other, PropertyAttributes otherAttrs)
{
    if (otherAttrs.hasEnumerable())
        attrs.setEnumerable(otherAttrs.isEnumerable());
    if (otherAttrs.hasConfigurable())
        attrs.setConfigurable(otherAttrs.isConfigurable());
    if (otherAttrs.hasWritable())
        attrs.setWritable(otherAttrs.isWritable());
    if (otherAttrs.type() == PropertyAttributes::Accessor) {
        attrs.setType(PropertyAttributes::Accessor);
        if (!other->value.isEmpty())
            value = other->value;
        if (!other->set.isEmpty())
            set = other->set;
    } else if (otherAttrs.type() == PropertyAttributes::Data){
        attrs.setType(PropertyAttributes::Data);
        value = other->value;
    }
}

}

Q_DECLARE_TYPEINFO(QV4::Property, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
