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

#ifndef Patternist_AtomicTypeDispatch_H
#define Patternist_AtomicTypeDispatch_H

#include <QSharedData>


QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class AnyAtomicType;
    class AnyURIType;
    class Base64BinaryType;
    class BooleanType;
    class DateTimeType;
    class DateType;
    class DayTimeDurationType;
    class DecimalType;
    class DoubleType;
    class DurationType;
    class FloatType;
    class GDayType;
    class GMonthDayType;
    class GMonthType;
    class GYearMonthType;
    class GYearType;
    class HexBinaryType;
    class IntegerType;
    class NOTATIONType;
    class QNameType;
    class SourceLocationReflection;
    class StringType;
    class SchemaTimeType;
    class UntypedAtomicType;
    class YearMonthDurationType;

    enum TypeOfDerivedInteger
    {
        TypeByte,
        TypeInt,
        TypeLong,
        TypeNegativeInteger,
        TypeNonNegativeInteger,
        TypeNonPositiveInteger,
        TypePositiveInteger,
        TypeShort,
        TypeUnsignedByte,
        TypeUnsignedInt,
        TypeUnsignedLong,
        TypeUnsignedShort
    };

    template<TypeOfDerivedInteger DerivedType> class DerivedIntegerType;

    enum TypeOfDerivedString
    {
        TypeString,
        TypeNormalizedString,
        TypeToken,
        TypeLanguage,
        TypeNMTOKEN,
        TypeName,
        TypeNCName,
        TypeID,
        TypeIDREF,
        TypeENTITY
    };

    template<TypeOfDerivedString DerivedType> class DerivedStringType;

    /**
     * @todo Documentation's missing:
     * - Add link to wikipedia's "multiple dispatch" and "visitor" page.
     * - Add link to http://www.eptacom.net/pubblicazioni/pub_eng/mdisp.html
     *
     * @defgroup Patternist_types_dispatch Atomic Type Dispatching
     */

    /**
     * @todo Docs missing
     *
     * @ingroup Patternist_types_dispatch
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AtomicTypeVisitorResult : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<AtomicTypeVisitorResult> Ptr;
        AtomicTypeVisitorResult() {}
        virtual ~AtomicTypeVisitorResult() {}
    };

    /**
     * @todo Docs missing
     *
     * @see ParameterizedAtomicTypeVisitor
     * @ingroup Patternist_types_dispatch
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AtomicTypeVisitor : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<AtomicTypeVisitor> Ptr;
        virtual ~AtomicTypeVisitor() {}

        virtual AtomicTypeVisitorResult::Ptr visit(const AnyAtomicType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const AnyURIType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const Base64BinaryType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const BooleanType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DateTimeType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DateType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DayTimeDurationType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DecimalType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DoubleType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DurationType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const FloatType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GDayType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GMonthDayType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GMonthType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GYearMonthType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GYearType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const HexBinaryType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const IntegerType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const NOTATIONType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const QNameType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const StringType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const SchemaTimeType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const UntypedAtomicType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const YearMonthDurationType *,
                                                   const SourceLocationReflection *const reflection) const = 0;
    };

    /**
     * @todo Docs missing
     *
     * @see AtomicTypeVisitor
     * @ingroup Patternist_types_dispatch
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ParameterizedAtomicTypeVisitor : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ParameterizedAtomicTypeVisitor> Ptr;
        virtual ~ParameterizedAtomicTypeVisitor() {}

        virtual AtomicTypeVisitorResult::Ptr visit(const AnyAtomicType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const AnyURIType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const Base64BinaryType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const BooleanType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DateTimeType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DateType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DayTimeDurationType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DecimalType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DoubleType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const DurationType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const FloatType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GDayType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GMonthDayType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GMonthType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GYearMonthType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const GYearType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const HexBinaryType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const IntegerType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const NOTATIONType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const QNameType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const StringType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const SchemaTimeType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const UntypedAtomicType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
        virtual AtomicTypeVisitorResult::Ptr visit(const YearMonthDurationType *, const qint16 param,
                                                   const SourceLocationReflection *const reflection) const = 0;
    };
}

QT_END_NAMESPACE

#endif
