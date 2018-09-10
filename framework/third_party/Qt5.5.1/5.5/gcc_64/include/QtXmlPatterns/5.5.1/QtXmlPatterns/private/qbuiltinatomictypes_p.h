/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

#ifndef Patternist_BuiltinAtomicTypes_H
#define Patternist_BuiltinAtomicTypes_H

#include <private/qatomiccasterlocators_p.h>
#include <private/qatomiccomparatorlocators_p.h>
#include <private/qbuiltinatomictype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Implements the type @c xs:anyAtomicType.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AnyAtomicType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<AnyAtomicType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * Overridden to return <tt>item()</tt>.
         *
         * @returns BuiltinTypes::item
         */
        virtual ItemType::Ptr xdtSuperType() const;

        /**
         * Overridden to return @c xs:anySimpleType.
         *
         * @returns BuiltinTypes::xsAnySimpleType
         */
        virtual SchemaType::Ptr wxsSuperType() const;

        /**
         * Overridden to return @c true, @c xs:anyAtomicType is abstract.
         *
         * @returns always @c true
         */
        virtual bool isAbstract() const;

    protected:
        friend class BuiltinTypes;
        AnyAtomicType();
    };

    /**
     * @short Implements the type @c xs:untypedAtomic.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class UntypedAtomicType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<UntypedAtomicType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        UntypedAtomicType();
    };

    /**
     * @short Implements the type @c xs:dateTime.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DateTimeType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DateTimeType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;

        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;
    protected:
        friend class BuiltinTypes;
        DateTimeType();
    };

    /**
     * @short Implements the type @c xs:date.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DateType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DateType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        DateType();
    };

    /**
     * @short Implements the type @c xs:time.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class SchemaTimeType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<SchemaTimeType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;

        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        SchemaTimeType();
    };

    /**
     * @short Implements the type @c xs:duration.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DurationType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DurationType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        DurationType();
    };

    /**
     * @short Implements the type @c xs:yearMonthDuration.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class YearMonthDurationType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<YearMonthDurationType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        YearMonthDurationType();
    };

    /**
     * @short Implements the type @c xs:dayTimeDuration.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DayTimeDurationType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DayTimeDurationType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        DayTimeDurationType();
    };

    /**
     * @short Implements the type @c xs:double.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DoubleType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DoubleType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        DoubleType();
    };

    /**
     * @short Implements the type @c xs:float.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class FloatType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<FloatType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        FloatType();
        friend class BuiltinTypes;
    };

    /**
     * @short Implements the type @c xs:decimal.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DecimalType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<DecimalType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        DecimalType();
    };

    /**
     * @short Implements the type @c xs:integer.
     *
     * IntegerType instances are used for representing all different xs:integer
     * types. The purpose of this is that xs:integer sub-types must use the
     * class, IntegerType, in order to use the correct behavior in call
     * dispatch situations. That is, all xs:integer sub-types must use the
     * same AtomicComparator as xs:integer itself uses, and that is achieved
     * this way.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class IntegerType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<IntegerType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        IntegerType(const AtomicType::Ptr &parentType,
                    const AtomicCasterLocator::Ptr &casterLocator);
    };

    template<TypeOfDerivedInteger derivedType>
    class DerivedIntegerType : public IntegerType
    {
    public:
        using IntegerType::accept;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &v,
                                                    const SourceLocationReflection *const r) const
        {
            return v->visit(this, r);
        }

        virtual QXmlName name(const NamePool::Ptr &np) const
        {
            switch(derivedType)
            {
                case TypeByte:                  return np->allocateQName(StandardNamespaces::xs, QLatin1String("byte"));
                case TypeInt:                   return np->allocateQName(StandardNamespaces::xs, QLatin1String("int"));
                case TypeLong:                  return np->allocateQName(StandardNamespaces::xs, QLatin1String("long"));
                case TypeNegativeInteger:       return np->allocateQName(StandardNamespaces::xs, QLatin1String("negativeInteger"));
                case TypeNonNegativeInteger:    return np->allocateQName(StandardNamespaces::xs, QLatin1String("nonNegativeInteger"));
                case TypeNonPositiveInteger:    return np->allocateQName(StandardNamespaces::xs, QLatin1String("nonPositiveInteger"));
                case TypePositiveInteger:       return np->allocateQName(StandardNamespaces::xs, QLatin1String("positiveInteger"));
                case TypeShort:                 return np->allocateQName(StandardNamespaces::xs, QLatin1String("short"));
                case TypeUnsignedByte:          return np->allocateQName(StandardNamespaces::xs, QLatin1String("unsignedByte"));
                case TypeUnsignedInt:           return np->allocateQName(StandardNamespaces::xs, QLatin1String("unsignedInt"));
                case TypeUnsignedLong:          return np->allocateQName(StandardNamespaces::xs, QLatin1String("unsignedLong"));
                case TypeUnsignedShort:         return np->allocateQName(StandardNamespaces::xs, QLatin1String("unsignedShort"));
            }

            Q_ASSERT_X(false, "DerivedIntegerType::name()", "Invalid value in instantiation.");
            return QXmlName();
        }

        virtual QString displayName(const NamePool::Ptr &np) const
        {
            return np->displayName(name(np));
        }

    protected:
        friend class BuiltinTypes;

        DerivedIntegerType(const AtomicType::Ptr &parentType,
                           const AtomicCasterLocator::Ptr &casterLoc) : IntegerType(parentType, casterLoc)
        {
        }

    };

    /**
     * @short Implements the type @c xs:gYearMonth.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class GYearMonthType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<GYearMonthType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        GYearMonthType();
    };

    /**
     * @short Implements the type @c xs:gYear.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class GYearType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<GYearType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        GYearType();
    };

    /**
     * @short Implements the type @c xs:gMonthDay.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class GMonthDayType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<GMonthDayType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        GMonthDayType();
    };

    /**
     * @short Implements the type @c xs:gDay.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class GDayType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<GDayType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        GDayType();
    };

    /**
     * @short Implements the type @c xs:gMonth.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class GMonthType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<GMonthType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        GMonthType();
    };

    /**
     * @short Implements the type @c xs:boolean.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class BooleanType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<BooleanType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        BooleanType();
    };

    /**
     * @short Implements the type @c xs:base64Binary.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Base64BinaryType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<Base64BinaryType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        Base64BinaryType();
    };

    /**
     * @short Implements the type @c xs:hexBinary.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class HexBinaryType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<HexBinaryType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        HexBinaryType();
    };

    /**
     * @short Implements the type @c xs:anyURI.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AnyURIType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<AnyURIType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        AnyURIType();
    };

    /**
     * @short Implements the type @c xs:QName.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class QNameType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<QNameType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        QNameType();
    };

    /**
     * Represents the xs:string type and all derived types of
     * xs:string, such as xs:token.
     *
     * StringType instances are used for representing all different string
     * types. The purpose of this is that xs:string sub-types must use the
     * class, StringType, in order to use the correct behavior in call
     * dispatch situations. That is, all xs:string sub-types must use the
     * same AtomicComparator as xs:string itself uses, and that is achieved
     * this way.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class StringType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<StringType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

    protected:
        friend class BuiltinTypes;
        StringType(const AtomicType::Ptr &parentType,
                   const AtomicCasterLocator::Ptr &casterLoc);
    };

    template<TypeOfDerivedString derivedType>
    class DerivedStringType : public StringType
    {
    public:
        using StringType::accept;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &v,
                                                    const SourceLocationReflection *const r) const
        {
            return v->visit(this, r);
        }

        virtual QXmlName name(const NamePool::Ptr &np) const
        {
            switch(derivedType)
            {
                case TypeString:            return np->allocateQName(StandardNamespaces::xs, QLatin1String("string"));
                case TypeNormalizedString:  return np->allocateQName(StandardNamespaces::xs, QLatin1String("normalizedString"));
                case TypeToken:             return np->allocateQName(StandardNamespaces::xs, QLatin1String("token"));
                case TypeLanguage:          return np->allocateQName(StandardNamespaces::xs, QLatin1String("language"));
                case TypeNMTOKEN:           return np->allocateQName(StandardNamespaces::xs, QLatin1String("NMTOKEN"));
                case TypeName:              return np->allocateQName(StandardNamespaces::xs, QLatin1String("Name"));
                case TypeNCName:            return np->allocateQName(StandardNamespaces::xs, QLatin1String("NCName"));
                case TypeID:                return np->allocateQName(StandardNamespaces::xs, QLatin1String("ID"));
                case TypeIDREF:             return np->allocateQName(StandardNamespaces::xs, QLatin1String("IDREF"));
                case TypeENTITY:            return np->allocateQName(StandardNamespaces::xs, QLatin1String("ENTITY"));
            }

            Q_ASSERT_X(false, "DerivedStringType::name()", "Invalid value in instantiation.");
            return QXmlName();
        }

        virtual QString displayName(const NamePool::Ptr &np) const
        {
            return np->displayName(name(np));
        }

    protected:
        friend class BuiltinTypes;

        DerivedStringType(const AtomicType::Ptr &parentType,
                          const AtomicCasterLocator::Ptr &casterLoc) : StringType(parentType, casterLoc)
        {
        }

    };

    /**
     * @short Implements the type @c xs:NOTATION.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class NOTATIONType : public BuiltinAtomicType
    {
    public:
        typedef QExplicitlySharedDataPointer<NOTATIONType> Ptr;

        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const reflection) const;
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const reflection) const;
        virtual QXmlName name(const NamePool::Ptr &np) const;
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * Overridden to return @c true, xs:NOTATION is abstract.
         *
         * @returns always @c true
         */
        virtual bool isAbstract() const;

    protected:
        friend class BuiltinTypes;
        NOTATIONType();
    };
}

QT_END_NAMESPACE

#endif
