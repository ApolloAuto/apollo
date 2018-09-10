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

#ifndef Patternist_AtomicComparator_H
#define Patternist_AtomicComparator_H

#include <QFlags>

#include <private/qitem_p.h>
#include <private/qatomictypedispatch_p.h>

QT_BEGIN_NAMESPACE

class QString;

namespace QPatternist
{

    /**
     * @short Base class for classes responsible of comparing two atomic values.
     *
     * This class is also known as the AtomicParrot.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT AtomicComparator : public AtomicTypeVisitorResult
    {
    public:
        AtomicComparator();
        virtual ~AtomicComparator();

        typedef QExplicitlySharedDataPointer<AtomicComparator> Ptr;

        /**
         * Identifies operators used in value comparisons.
         *
         * The enum values are bit-significant.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#id-value-comparisons">W3C XML Path
         * Language (XPath) 2.0, 3.5.1 Value Comparisons</a>
         */
        enum Operator
        {
            /**
             * Operator <tt>eq</tt> and <tt>=</tt>.
             */
            OperatorEqual           = 1,

            /**
             * Operator <tt>ne</tt> and <tt>!=</tt>.
             */
            OperatorNotEqual        = 1 << 1,

            /**
             * Operator <tt>gt</tt> and <tt>\></tt>.
             */
            OperatorGreaterThan     = 1 << 2,

            /**
             * Operator <tt>lt</tt> and <tt>\<</tt>.
             */
            OperatorLessThan        = 1 << 3,

            /**
             * One of the operators we use for sorting. The only difference from
             * OperatorLessThan is that it sees NaN as ordered and smaller than
             * other numbers.
             */
            OperatorLessThanNaNLeast    = 1 << 4,

            /**
             * One of the operators we use for sorting. The only difference from
             * OperatorLessThanLeast is that it sees NaN as ordered and larger than
             * other numbers.
             */
            OperatorLessThanNaNGreatest    = 1 << 5,

            /**
             * Operator <tt>ge</tt> and <tt>\>=</tt>.
             */
            OperatorGreaterOrEqual  = OperatorEqual | OperatorGreaterThan,

            /**
             * Operator <tt>le</tt> and <tt>\<=</tt>.
             */
            OperatorLessOrEqual     = OperatorEqual | OperatorLessThan
        };

        typedef QFlags<Operator> Operators;

        /**
         * Signifies the result of a value comparison. This is used for value comparisons,
         * and in the future likely also for sorting.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#id-value-comparisons">W3C XML Path
         * Language (XPath) 2.0, 3.5.1 Value Comparisons</a>
         */
        enum ComparisonResult
        {
            LessThan     = 1,
            Equal        = 2,
            GreaterThan  = 4,
            Incomparable = 8
        };

        /**
         * Compares @p op1 and @p op2 and determines the relationship between the two. This
         * is used for sorting and comparisons. The implementation performs an assert crash,
         * and must therefore be re-implemented if comparing the relevant values should be
         * possible.
         *
         * @param op1 the first operand
         * @param op the operator. How a comparison is carried out shouldn't depend on what the
         * operator is, but in some cases it is of interest.
         * @param op2 the second operand
         */
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        /**
         * Determines whether @p op1 and @p op2 are equal. It is the same as calling compare()
         * and checking whether the return value is Equal, but since comparison testing is such
         * a common operation, this specialized function exists.
         *
         * @returns true if @p op1 and @p op2 are equal.
         *
         * @param op1 the first operand
         * @param op2 the second operand
         */
        virtual bool equals(const Item &op1,
                            const Item &op2) const = 0;

        /**
         * Identifies the kind of comparison.
         */
        enum ComparisonType
        {
            /**
             * Identifies a general comparison; operator @c =, @c >, @c <=, and so on.
             */
            AsGeneralComparison = 1,

            /**
             * Identifies a value comparison; operator @c eq, @c lt, @c le, and so on.
             */
            AsValueComparison
        };

        /**
         * Utility function for getting the lexical representation for
         * the comparison operator @p op. Depending on the @p type argument,
         * the string returned is either a general comparison or a value comparison
         * operator.
         *
         * @param op the operator which the display name should be determined for.
         * @param type signifies whether the returned display name should be for
         * a value comparison or a general comparison. For example, if @p op is
         * OperatorEqual and @p type is AsValueComparision, "eq" is returned.
         */
        static QString displayName(const AtomicComparator::Operator op,
                                   const ComparisonType type);

    };
    Q_DECLARE_OPERATORS_FOR_FLAGS(AtomicComparator::Operators)
}

QT_END_NAMESPACE

#endif
