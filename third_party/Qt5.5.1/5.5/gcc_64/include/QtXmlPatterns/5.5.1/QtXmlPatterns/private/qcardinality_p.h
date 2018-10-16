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

#ifndef Patternist_Cardinality_H
#define Patternist_Cardinality_H

#include <QtCore/QtGlobal>

QT_BEGIN_NAMESPACE

class QString;

namespace QPatternist
{
    /**
     * @short Represents a cardinality, a possible , often represented by occurrence indicators.
     *
     * As opposed to the cardinality concept in the XQuery/XPath specifications, which
     * only allows cardinalities to be expressed with kleene operators, this representation
     * allows ranges. For example, the cardinality 10-11, describes a sequence containing
     * ten or eleven items, inclusive.
     *
     * @ingroup Patternist_types
     * @see ItemType
     * @see SequenceType
     * @see <a href="http://www.w3.org/TR/xpath20/#prod-xpath-SequenceType">XML Path Language
     * (XPath) 2.0, The EBNF grammar for SequenceType</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Cardinality
    {
    public:
        /**
         * This integer type, is what Cardinality uses for representing its ranges.
         */
        typedef qint32 Count;

        /**
         * Used with displayName(), and specifies
         * how a display name for a Cardinality should be.
         */
        enum CustomizeDisplayName
        {
            /**
             * Includes a describing string in the return value of displayName().
             */
            IncludeExplanation  = 1,

            /**
             * Excludes a describing string in the return value of displayName().
             */
            ExcludeExplanation
        };

        /**
         * A traditional copy constructor. This Cardinality becomes identical
         * to @p other.
         */
        inline Cardinality(const Cardinality &other) : m_min(other.m_min),
                                                       m_max(other.m_max)
        {
        }

        /**
         * This default constructor constructs an invalid Cardinality. Using
         * its operators and members yields undefined results. A value must
         * first be assigned to it by creating a Cardinality with fromRange(), fromCount(),
         * or one of the predefined cardinalities such as empty() or oneOrMore().
         */
        inline Cardinality() : m_min(-1), m_max(0)
        {
        }

        /**
         * The cardinality assigned to the exprssion <tt>()</tt>, formally speaking. The
         * cardinality part of <tt>empty-sequence()</tt>.
         */
        static inline Cardinality empty()
        {
            return Cardinality(0, 0);
        }

        /**
         * The cardinality implicitly specified in for example the sequence type
         * <tt>item()</tt>. It has no kleene operator.
         */
        static inline Cardinality exactlyOne()
        {
            return Cardinality(1, 1);
        }

        /**
         * Allows both no item, as in empty(), and exactlyOne(). Represented
         * by the kleene operator <tt>?</tt>.
         */
        static inline Cardinality zeroOrOne()
        {
            return Cardinality(0, 1);
        }

        /**
         * Allows any amount. This is therefore the widest, an unconstrained
         * cardinality. Represented by the kleene operator <tt>*</tt>.
         */
        static inline Cardinality zeroOrMore()
        {
            return Cardinality(0, -1);
        }

        /**
         * Allows one or more. Represented by the kleene operator <tt>+</tt>.
         */
        static inline Cardinality oneOrMore()
        {
            return Cardinality(1, -1);
        }

        /**
         * Allows one or more. This cardinality has no kleene operator and is used
         * by the implementation in order to be able to know when a cardinality
         * that at amximum allows one, is exceeded.
         */
        static inline Cardinality twoOrMore()
        {
            return Cardinality(2, -1);
        }

       /**
         * Determines the cardinality from the count of a sequence. For example, if
         * @p count is 11, a Cardinality is returned that allows at minimum and maximum
         * 11 items.
         *
         * @p count must be positive or zero. If it is not, the result is undefined.
         * When debugging is enabled, a Q_ASSERT() macro ensures this.
         */
        static inline Cardinality fromCount(const Count count)
        {
            Q_ASSERT_X(count > -1, Q_FUNC_INFO,
                       "A count smaller than 0 makes no sense.");
            return Cardinality(count, count);
        }

        /**
         * Creates a Cardinality that allows @p minimum and @p maximum
         * items, inclusive.
         *
         * If @p maximum is -1, it signals infinity.
         *
         * If you before hand knows that a predefined Cardinality is needed,
         * remember to use one of the factory functions empty(), zeroOrOne(),
         * exactlyOne(), oneOrMore() or zeroOrMore(), since they improves
         * readability, are safer, and slightly faster.
         */
        static inline Cardinality fromRange(const Count minimum, const Count maximum)
        {
            Q_ASSERT_X(minimum > -1, Q_FUNC_INFO,
                       "minimum should never be less than 0.");
            Q_ASSERT_X(minimum <= maximum || maximum == -1, Q_FUNC_INFO,
                       "minimum cannot be larger than maximum.");

            return Cardinality(minimum, maximum);
        }

        static inline Cardinality fromExact(const Count count)
        {
            Q_ASSERT(count >= 0);
            return Cardinality(count, count);
        }

        /**
         * @returns the minimum amount of items this Cardinality allows. For example,
         * for zeroOrOne() is 0 returned.
         */
        inline Count minimum() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality are invalid.");
            return m_min;
        }

        /**
         * @returns the maximum amount of items this Cardinality allows. For example,
         * for zeroOrOne() is 1 returned.
         */
        inline Count maximum() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality are invalid.");
            return m_max;
        }

        /**
         * @returns @c true if this Cardinality allows one or more items. For example, for
         * zeroOrOne() is @c false returned, while for zeroOrMore() is @c true returned.
         */
        inline bool allowsMany() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality are invalid.");
            return m_max == -1 || m_max > 1;
        }

        /**
         * @returns @c true if this Cardinality allows no items. For example, for
         * zeroOrOne() is @c true returned, while for oneOrMore() is @c false returned.
         */
        inline bool allowsEmpty() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality are invalid.");
            return m_min == 0;
        }

        /**
         * Maps directly to Formal Semantics' @c aggregate_quantifier function.
         *
         * @returns zeroOrOne() if this Cardinality allows the empty sequence, otherwise exactlyOne()
         * @see <a href="http://www.w3.org/TR/xquery-semantics/#jd_quantifier">XQuery 1.0 and
         * XPath 2.0 Formal Semantics, The function quantifier()</a>
         */
        inline Cardinality toWithoutMany() const
        {
            return m_min == 0 ? Cardinality(0, 1)
                              : Cardinality(1, 1);
        }

        /**
         * Determines whether all the possible outcomes represented by @p other,
         * will always match this Cardinality. For example, if this Cardinality
         * is oneOrMore(), @c true will be returned if @p other is exactlyOne(), but
         * false if @p other is zeroOrOne().
         */
        inline bool isMatch(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");
            if(other.m_min < m_min)
                return false;
            else
            { /* Ok, we now know the minimum will always be ok. */
                if(m_max == -1)
                    return true; /* We allow infinite, so anything can match. */
                else if(other.m_max == -1)
                    return false; /* other allows infinity, while we don't. */
                else
                   return m_max >= other.m_max;
            }
        }

        /**
         * Determines whether at least one of the possible outcomes represented by @p other,
         * can match this Cardinality. For example, if this Cardinality
         * is oneOrMore(), @c true will be returned if @p other is exactlyOne() or zeroOrOne().
         */
        inline bool canMatch(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");
            if(m_max == -1)
                return m_min <= other.m_min || other.m_max >= m_min || other.m_max == -1;
            else
            {
                if(m_max == other.m_min)
                    return true;
                else if(m_max > other.m_min)
                    return other.m_max >= m_min || other.m_max == -1;
                else /* m_max < other.m_min */
                    return false;
            }
        }

        /**
         * @returns @c true if this Cardinality is empty, the <tt>empty-sequence()</tt>, otherwise
         * @c false.
         */
        inline bool isEmpty() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality is invalid.");
            return m_min == 0 && m_max == 0;
        }

        /**
         * @returns @c true if this Cardinality is zero-or-one, <tt>?</tt>, otherwise
         * @c false.
         */
        inline bool isZeroOrOne() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality is invalid.");
            return m_min == 0 && m_max == 1;
        }

        /**
         * @returns @c true if this Cardinality only allows exactly one item, otherwise
         * @c false.
         */
        inline bool isExactlyOne() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality is invalid.");
            return m_min == 1 && m_max == 1;
        }

        /**
         * @returns @c true if this Cardinality only allows one or more items, otherwise
         * @c false.
         */
        inline bool isOneOrMore() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality is invalid.");
            return m_min > 0 && (m_max == -1 || m_max >= 1);
        }

        /**
         * Determines whether this Cardinality only allows a specific length. For example,
         * empty() and exactlyOne() are exact, but oneOrMore() or zeroOrOne() is not.
         */
        inline bool isExact() const
        {
            Q_ASSERT_X(m_min != -1, Q_FUNC_INFO, "The cardinality is invalid.");
            return m_min == m_max;
        }

        /**
         * Returns a string representation of this Cardinality.
         *
         * If @p explain is ExcludeExplanation the kleene operator is returned. For example, if
         * the Cardinality is zeroOrOne, is "?" returned.
         *
         * If explain is IncludeExplanation a string more suited for human interpretation is returned,
         * which is appropriately translated. For example, when the locale is English and
         * this Cardinality being zeroOrOne, then is 'zero or one("?")' returned.
         *
         * Typically, passing ExcludeExplanation is useful when generating function
         * signatures and the like, while passing IncludeExplanation
         * is suitable appropriate when generating error messages.
         *
         * @returns a string representation for this Cardinality.
         */
        QString displayName(const CustomizeDisplayName explanation) const;

        /**
         * Computes the Cardinality that comprises this Cardinality as well as @p other. For
         * example, if this Cardinality is zeroOrOne() and @p other is oneOrMore(), then
         * is zeroOrMore() returned.
         */
        inline Cardinality operator|(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");
            if(m_max == -1 || other.m_max == -1)
                return Cardinality(qMin(m_min, other.m_min), -1);
            else
                return Cardinality(qMin(m_min, other.m_min), qMax(m_max, other.m_max));
        }

        /**
         * Behaves as operator|() but assigns the result to this Cardinality.
         */
        inline Cardinality &operator|=(const Cardinality &other)
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");
            m_min = qMin(m_min, other.m_min);

            if(m_max == -1)
                return *this;
            else if(other.m_max == -1)
                m_max = -1;
            else
                m_max = qMax(m_max, other.m_max);

            return *this;
        }

        /**
         * Computes the intersection of this Cardinality and @p other, and returns
         * the result. For example, the intersection between zeroOrOne() and
         * oneOrMore() is exactlyOne().
         *
         * If no intersection exists, such as the case in empty() and exactlyOne(), then
         * is a default constructed Cardinality is returned. That is, an invalid Cardinality.
         */
        inline Cardinality operator&(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");

            if(m_max < other.m_min) /* No intersection. */
                return empty();

            const Count min = qMax(m_min, other.m_min);

            if(m_max == -1)
                return Cardinality(min, other.m_max);
            else if(other.m_max == -1)
                return Cardinality(min, m_max);
            else
                return Cardinality(min, qMin(m_max, other.m_max));
        }

        /**
         * Adds two cardinalities, as if two sequences represented by them were concatenated.
         * For example, if this Cardinality allows the range 6-8 and @p other allows
         * 0-1, the return Cardinality has a range of 6-9.
         *
         * @returns the result of the comparison.
         */
        inline Cardinality operator+(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO, "One of the cardinalities are invalid.");
            if(m_max == -1 || other.m_max == -1)
                return Cardinality(m_min + other.m_min, -1);
            else
                return Cardinality(m_min + other.m_min, m_max + other.m_max);
        }

        /**
         * Behaves as operator+() but assigns the result to this Cardinality.
         */
        inline Cardinality &operator+=(const Cardinality &other)
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO,
                       "One of the cardinalities are invalid.");
            m_min += other.m_min;

            if(m_max == -1)
                return *this;
            if(other.m_max == -1)
                m_max = -1;
            else
                m_max += other.m_max;

            return *this;
        }

        /**
         * Multiplies this Cardinality with @p other, and returns the result. The minimum and maximum
         * of each Cardinality is multiplied such that the new Cardinality represents the possible
         * range of the two sequences being multiplied, length-wise. For example the Cardinality
         * 4, 5 multiplied with 2, 3 becomes 8, 15.
         */
        inline Cardinality operator*(const Cardinality &other) const
        {
            Q_ASSERT_X(m_min != -1 && other.m_min != -1, Q_FUNC_INFO,
                       "One of the cardinalities are invalid.");
            if(m_max == -1 || other.m_max == -1)
                return Cardinality(m_min * other.m_min, -1);
            else
                return Cardinality(m_min * other.m_min, m_max * other.m_max);
        }

        /**
         * A traditional assignment operator. Behaves as assignment
         * operators typically do.
         */
        inline Cardinality &operator=(const Cardinality &other)
        {
            Q_ASSERT_X(this != &other, Q_FUNC_INFO, "Assigning to oneself makes no sense.");
            m_min = other.m_min;
            m_max = other.m_max;
            return *this;
        }

        /**
         * Determines whether @p other is equal to this Cardinality.
         *
         * For example, empty() is equal to empty(), but zeroOrOne()
         * is not equal to exactlyOne().
         *
         * @returns @c true if @p other is equal to this Cardinality.
         */
        inline bool operator==(const Cardinality &other) const
        {
            return m_min == other.m_min &&
                   m_max == other.m_max;
        }

        /**
         * @returns the opposite of operator==()
         */
        inline bool operator!=(const Cardinality &other) const
        {
            return m_min != other.m_min ||
                   m_max != other.m_max;
        }

    private:
        inline Cardinality(const Count min, const Count max) : m_min(min),
                                                               m_max(max)
        {
        }

        Count m_min;
        Count m_max;
    };
}

Q_DECLARE_TYPEINFO(QPatternist::Cardinality, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
