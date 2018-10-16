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

#ifndef Patternist_AtomicComparators_H
#define Patternist_AtomicComparators_H

#include <private/qabstractfloat_p.h>
#include <private/qatomiccomparator_p.h>
#include <private/qprimitives_p.h>

/**
 * @file
 * @short Contains all the classes implementing comparisons between atomic values.
 */

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Performs case @em sensitive string comparison
     * between @c xs:anyUri, @c xs:string, and @c xs:untypedAtomic.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class StringComparator : public AtomicComparator
    {
    public:
        /**
         * Compares two strings, and returns the appropriate AtomicComparator::ComparisonResult enum. This
         * is a bit simplified version of string comparison as defined in the XPath specifications,
         * since this does not take any string collations into account(which an implementation neither
         * is required to do).
         *
         * @see <a href="http://www.w3.org/TR/xpath-functions/#string-compare">XQuery 1.0 and XPath
         * 2.0 Functions and Operators, 7.3 ValueComparison::Equality and Comparison of Strings</a>
         */
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        /**
         * Compares two strings, and returns @c true if they are considered equal as per
         * an ordinary string comparison. In other words, this is an implementation with
         * the Unicode code point collation.
         *
         * @see <a href="http://www.w3.org/TR/xpath-functions/#string-compare">XQuery 1.0 and XPath
         * 2.0 Functions and Operators, 7.3 ValueComparison::Equality and Comparison of Strings</a>
         */
        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Performs case @em insensitive string comparison
     * between @c xs:anyUri, @c xs:string, and @c xs:untypedAtomic.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class CaseInsensitiveStringComparator : public AtomicComparator
    {
    public:
        /**
         * Converts both string values to upper case and afterwards compare them.
         */
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        /**
         * Converts both string values case insensitively.
         */
        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:base64Binary and @c xs:hexBinary values.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class BinaryDataComparator : public AtomicComparator
    {
    public:
        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:boolean values.
     *
     * This is done via the object's Boolean::evaluteEBV() function.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class BooleanComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:double values.
     *
     * @todo Add docs about numeric promotion
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AbstractFloatComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:double values for the purpose of sorting.
     *
     * @todo Add docs about numeric promotion
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    template<const AtomicComparator::Operator t_op>
    class AbstractFloatSortComparator : public AbstractFloatComparator
    {
    public:
        virtual ComparisonResult compare(const Item &o1,
                                         const AtomicComparator::Operator op,
                                         const Item &o2) const
        {
            Q_ASSERT_X(t_op == OperatorLessThanNaNLeast || t_op == OperatorLessThanNaNGreatest, Q_FUNC_INFO,
                       "Can only be instantiated with those two.");
            Q_ASSERT(op == t_op);
            Q_UNUSED(op); /* Needed when building in release mode. */

            const xsDouble v1 = o1.template as<Numeric>()->toDouble();
            const xsDouble v2 = o2.template as<Numeric>()->toDouble();

            if(qIsNaN(v1) && !qIsNaN(v2))
                return t_op == OperatorLessThanNaNLeast ? LessThan : GreaterThan;
            if(!qIsNaN(v1) && qIsNaN(v2))
                return t_op == OperatorLessThanNaNLeast ? GreaterThan : LessThan;

            if(Double::isEqual(v1, v2))
                return Equal;
            else if(v1 < v2)
                return LessThan;
            else
                return GreaterThan;
        }

    };

    /**
     * @short Compares @c xs:decimal values.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DecimalComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:integer values.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class IntegerComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;

        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares @c xs:QName values.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class QNameComparator : public AtomicComparator
    {
    public:
        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares sub-classes of AbstractDateTime.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AbstractDateTimeComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;
        virtual bool equals(const Item &op1,
                            const Item &op2) const;
    };

    /**
     * @short Compares sub-classes of AbstractDuration.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AbstractDurationComparator : public AtomicComparator
    {
    public:
        virtual ComparisonResult compare(const Item &op1,
                                         const AtomicComparator::Operator op,
                                         const Item &op2) const;
        virtual bool equals(const Item &op1,
                            const Item &op2) const;

    private:
        static inline QDateTime addDurationToDateTime(const QDateTime &dateTime,
                                                      const AbstractDuration *const duration);
    };
}

QT_END_NAMESPACE

#endif
