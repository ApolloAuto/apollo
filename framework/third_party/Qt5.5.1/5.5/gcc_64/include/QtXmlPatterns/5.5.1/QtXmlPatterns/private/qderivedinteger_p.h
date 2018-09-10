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

#ifndef Patternist_DerivedInteger_H
#define Patternist_DerivedInteger_H

#include <private/qbuiltintypes_p.h>
#include <private/qinteger_p.h>
#include <private/qpatternistlocale_p.h>
#include <private/qvalidationerror_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @relates DerivedInteger
     */
    enum DerivedIntegerLimitsUsage
    {
        None            = 1,
        LimitUpwards    = 2,
        LimitDownwards  = 4,
        LimitBoth       = LimitUpwards | LimitDownwards
    };

    enum
    {
        IgnorableSignedValue = 0,
        IgnorableUnsignedValue = 0
    };

    template<TypeOfDerivedInteger DerivedType> class DerivedInteger;

    template<TypeOfDerivedInteger DerivedType> class DerivedIntegerDetails;

    template<>
    class DerivedIntegerDetails<TypeByte>
    {
    private:
        friend class DerivedInteger<TypeByte>;
        typedef qint8                           StorageType;
        typedef xsInteger                       TemporaryStorageType;
        static const StorageType                maxInclusive = 127;
        static const StorageType                minInclusive = -128;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeInt>
    {
    private:
        friend class DerivedInteger<TypeInt>;
        typedef qint32                          StorageType;
        typedef xsInteger                       TemporaryStorageType;
        static const StorageType                maxInclusive = Q_INT64_C(2147483647);
        static const StorageType                minInclusive = Q_INT64_C(-2147483648);
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeLong>
    {
    private:
        friend class DerivedInteger<TypeLong>;
        typedef qint64                          StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = Q_INT64_C(9223372036854775807);

        /**
         * This messy arithmetic expression ensures that we don't get a warning
         * on neither GCC nor MSVC.
         */
        static const StorageType                minInclusive = -(Q_INT64_C(9223372036854775807)) - 1;

        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeNegativeInteger>
    {
    private:
        friend class DerivedInteger<TypeNegativeInteger>;
        typedef xsInteger                       StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = -1;
        static const StorageType                minInclusive = IgnorableSignedValue;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitUpwards;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeNonNegativeInteger>
    {
    private:
        friend class DerivedInteger<TypeNonNegativeInteger>;
        typedef xsInteger                       StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = IgnorableSignedValue;
        static const StorageType                minInclusive = 0;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitDownwards;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeNonPositiveInteger>
    {
    private:
        friend class DerivedInteger<TypeNonPositiveInteger>;
        typedef xsInteger                       StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = 0;
        static const StorageType                minInclusive = IgnorableSignedValue;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitUpwards;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypePositiveInteger>
    {
    private:
        friend class DerivedInteger<TypePositiveInteger>;
        typedef xsInteger                       StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = IgnorableSignedValue;
        static const StorageType                minInclusive = 1;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitDownwards;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeShort>
    {
    private:
        friend class DerivedInteger<TypeShort>;
        typedef qint16                          StorageType;
        typedef xsInteger                       TemporaryStorageType;
        static const StorageType                maxInclusive = 32767;
        static const StorageType                minInclusive = -32768;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeUnsignedByte>
    {
    private:
        friend class DerivedInteger<TypeUnsignedByte>;
        typedef quint8                          StorageType;
        typedef qint64                          TemporaryStorageType;
        static const StorageType                maxInclusive = 255;
        static const StorageType                minInclusive = 0;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeUnsignedInt>
    {
    private:
        friend class DerivedInteger<TypeUnsignedInt>;
        typedef quint32                         StorageType;
        typedef qint64                          TemporaryStorageType;
        static const StorageType                maxInclusive = Q_UINT64_C(4294967295);
        static const StorageType                minInclusive = 0;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeUnsignedLong>
    {
    private:
        friend class DerivedInteger<TypeUnsignedLong>;
        typedef quint64                         StorageType;
        typedef StorageType                     TemporaryStorageType;
        static const StorageType                maxInclusive = Q_UINT64_C(18446744073709551615);
        static const StorageType                minInclusive = 0;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    template<>
    class DerivedIntegerDetails<TypeUnsignedShort>
    {
    private:
        friend class DerivedInteger<TypeUnsignedShort>;
        typedef quint16                         StorageType;
        typedef qint64                          TemporaryStorageType;
        static const StorageType                maxInclusive = 65535;
        static const StorageType                minInclusive = 0;
        static const DerivedIntegerLimitsUsage  limitsUsage = LimitBoth;

        /**
         * Disable the default constructor.
         */
        DerivedIntegerDetails() {}

        Q_DISABLE_COPY(DerivedIntegerDetails)
    };

    /**
     * @short Represents instances of derived @c xs:integer types, such as @c
     * xs:byte.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_xdm
     */
    template<TypeOfDerivedInteger DerivedType>
    class DerivedInteger : public Numeric
    {
    private:
        typedef typename DerivedIntegerDetails<DerivedType>::StorageType StorageType;
        typedef typename DerivedIntegerDetails<DerivedType>::TemporaryStorageType TemporaryStorageType;

        static const StorageType                maxInclusive        = DerivedIntegerDetails<DerivedType>::maxInclusive;
        static const StorageType                minInclusive        = DerivedIntegerDetails<DerivedType>::minInclusive;
        static const DerivedIntegerLimitsUsage  limitsUsage         = DerivedIntegerDetails<DerivedType>::limitsUsage;

        const StorageType m_value;

        inline DerivedInteger(const StorageType num) : m_value(num)
        {
        }

        /**
         * By refactoring out the simple comparison below into a template
         * function, we avoid the warning "warning: comparison of unsigned expression < 0 is always false" with gcc
         * when the class is instantiated with TypeUnsignedLong. The warning is
         * a false positive since we check wehther LimitUpwards is set before
         * instantiating.
         *
         * This template function exists for no other reason. */
        template<typename A, typename B>
        static bool lessThan(const A &a, const B &b)
        {
            return a < b;
        }

        /**
         * This function exists for the same reason that lessThan() do.
         */
        template<typename A, typename B>
        static bool largerOrEqual(const A &a, const B &b)
        {
            return qint64(a) >= b;
        }

    public:

        static ItemType::Ptr itemType()
        {
            switch(DerivedType)
            {
                case TypeByte:                  return BuiltinTypes::xsByte;
                case TypeInt:                   return BuiltinTypes::xsInt;
                case TypeLong:                  return BuiltinTypes::xsLong;
                case TypeNegativeInteger:       return BuiltinTypes::xsNegativeInteger;
                case TypeNonNegativeInteger:    return BuiltinTypes::xsNonNegativeInteger;
                case TypeNonPositiveInteger:    return BuiltinTypes::xsNonPositiveInteger;
                case TypePositiveInteger:       return BuiltinTypes::xsPositiveInteger;
                case TypeShort:                 return BuiltinTypes::xsShort;
                case TypeUnsignedByte:          return BuiltinTypes::xsUnsignedByte;
                case TypeUnsignedInt:           return BuiltinTypes::xsUnsignedInt;
                case TypeUnsignedLong:          return BuiltinTypes::xsUnsignedLong;
                case TypeUnsignedShort:         return BuiltinTypes::xsUnsignedShort;
            }

            Q_ASSERT(false);
            return ItemType::Ptr();
        }

        static AtomicValue::Ptr fromValue(const NamePool::Ptr &np, const TemporaryStorageType num)
        {
            /* If we use minInclusive when calling lessThan(), we for some
             * reason get a linker error with GCC. Using this temporary
             * variable solves it. */
            const StorageType minimum = minInclusive;
            // MSVC2010 complains that this is initialised but not referenced.
            Q_UNUSED(minimum)

            if((limitsUsage & LimitUpwards) &&
               num > maxInclusive)
            {
                return ValidationError::createError(QtXmlPatterns::tr(
                    "Value %1 of type %2 exceeds maximum (%3).")
                    .arg(QPatternist::formatData(static_cast<xsInteger>(num)))
                    .arg(formatType(np, itemType()))
                    .arg(QPatternist::formatData(static_cast<xsInteger>(maxInclusive))));
            }
            else if((limitsUsage & LimitDownwards) &&
                    lessThan(num, minimum))
            {
                return ValidationError::createError(QtXmlPatterns::tr(
                    "Value %1 of type %2 is below minimum (%3).")
                    .arg(QPatternist::formatData(static_cast<xsInteger>(num)))
                    .arg(formatType(np, itemType()))
                    .arg(QPatternist::formatData(static_cast<xsInteger>(minInclusive))));
            }
            else
                return AtomicValue::Ptr(new DerivedInteger(num));
        }

        static AtomicValue::Ptr fromValueUnchecked(const TemporaryStorageType num)
        {
            return AtomicValue::Ptr(new DerivedInteger(num));
        }

        /**
         * Constructs an instance from the lexical
         * representation @p strNumeric.
         */
        static AtomicValue::Ptr fromLexical(const NamePool::Ptr &np, const QString &strNumeric)
        {
            bool conversionOk = false;
            TemporaryStorageType num;

            /* Depending on the type, we need to call different conversion
             * functions on QString. */
            switch(DerivedType)
            {
                case TypeUnsignedLong:
                {
                    /* Qt decides to flag '-' as invalid, so remove it before. */
                    if(strNumeric.contains(QLatin1Char('-')))
                    {
                        num = QString(strNumeric).remove(QLatin1Char('-')).toULongLong(&conversionOk);

                        if(num != 0)
                            conversionOk = false;
                    }
                    else
                        num = strNumeric.toULongLong(&conversionOk);

                    break;
                }
                default:
                {
                    num = strNumeric.toLongLong(&conversionOk);
                    break;
                }
            }

            if(conversionOk)
                return fromValue(np, num);
            else
                return ValidationError::createError();
        }

        inline StorageType storedValue() const
        {
            return m_value;
        }

        /**
         * Determines the Effective %Boolean Value of this number.
         *
         * @returns @c false if the number is 0, otherwise @c true.
         */
        bool evaluateEBV(const QExplicitlySharedDataPointer<DynamicContext> &) const
        {
            return m_value != 0;
        }

        virtual QString stringValue() const
        {
            return QString::number(m_value);
        }

        virtual ItemType::Ptr type() const
        {
            return itemType();
        }

        virtual xsDouble toDouble() const
        {
            return static_cast<xsDouble>(m_value);
        }

        virtual xsInteger toInteger() const
        {
            return m_value;
        }

        virtual xsFloat toFloat() const
        {
            return static_cast<xsFloat>(m_value);
        }

        virtual xsDecimal toDecimal() const
        {
            return static_cast<xsDecimal>(m_value);
        }

        virtual Numeric::Ptr round() const
        {
            /* xs:integerS never have a mantissa. */
            return Numeric::Ptr(static_cast<Numeric *>(const_cast<AtomicValue *>(Integer::fromValue(m_value).asAtomicValue())));
        }

        virtual Numeric::Ptr roundHalfToEven(const xsInteger) const
        {
            return Numeric::Ptr(static_cast<Numeric *>(const_cast<AtomicValue *>(Integer::fromValue(m_value).asAtomicValue())));
        }

        virtual Numeric::Ptr floor() const
        {
            return Numeric::Ptr(static_cast<Numeric *>(const_cast<AtomicValue *>(Integer::fromValue(m_value).asAtomicValue())));
        }

        virtual Numeric::Ptr ceiling() const
        {
            return Numeric::Ptr(static_cast<Numeric *>(const_cast<AtomicValue *>(Integer::fromValue(m_value).asAtomicValue())));
        }

        virtual Numeric::Ptr abs() const
        {
            /* We unconditionally create an Integer even if we're a positive
             * value, because one part of this is the type change to
             * xs:integer.
             *
             * We've manually inlined qAbs() and invoke xsInteger's
             * constructor. The reason being that we other gets truncation down
             * to StorageType. See for instance XQTS test case absint1args-1. */
            return Numeric::Ptr(static_cast<Numeric *>(const_cast<AtomicValue *>(Integer::fromValue(largerOrEqual(m_value, 0) ? xsInteger(m_value) : -xsInteger(m_value)).asAtomicValue())));
        }

        /**
         * @returns always @c false, @c xs:DerivedInteger doesn't have
         * not-a-number in its value space.
         */
        virtual bool isNaN() const
        {
            return false;
        }

        /**
         * @returns always @c false, @c xs:DerivedInteger doesn't have
         * infinity in its value space.
         */
        virtual bool isInf() const
        {
            return false;
        }

        virtual Item toNegated() const
        {
            return Integer::fromValue(-xsInteger(m_value));
        }

        virtual bool isSigned() const
        {
            switch(DerivedType)
            {
                /* Fallthrough all these. */
                case TypeByte:
                case TypeInt:
                case TypeLong:
                case TypeNegativeInteger:
                case TypeNonNegativeInteger:
                case TypeNonPositiveInteger:
                case TypePositiveInteger:
                case TypeShort:
                    return true;
                /* Fallthrough all these. */
                case TypeUnsignedByte:
                case TypeUnsignedInt:
                case TypeUnsignedLong:
                case TypeUnsignedShort:
                    return false;
            }
            return false;
        }

        virtual qulonglong toUnsignedInteger() const
        {
            switch(DerivedType)
            {
                /* Fallthrough all these. */
                case TypeByte:
                case TypeInt:
                case TypeLong:
                case TypeNegativeInteger:
                case TypeNonNegativeInteger:
                case TypeNonPositiveInteger:
                case TypePositiveInteger:
                case TypeShort:
                    Q_ASSERT_X(false, Q_FUNC_INFO,
                               "It makes no sense to call this function, see Numeric::toUnsignedInteger().");
                /* Fallthrough all these. */
                case TypeUnsignedByte:
                case TypeUnsignedInt:
                case TypeUnsignedLong:
                case TypeUnsignedShort:
                    return m_value;
            }
            return 0;
        }

    };
}

QT_END_NAMESPACE

#endif
