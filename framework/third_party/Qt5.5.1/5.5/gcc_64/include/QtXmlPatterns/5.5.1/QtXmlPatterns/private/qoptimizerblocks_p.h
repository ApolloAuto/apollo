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

#ifndef Patternist_OptimizerBlocks_H
#define Patternist_OptimizerBlocks_H

#include <private/qatomiccomparator_p.h>
#include <private/qexpression_p.h>
#include <private/qoptimizerframework_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Identifies Expression instances by their Expression::id().
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ByIDIdentifier : public ExpressionIdentifier
    {
    public:
        ByIDIdentifier(const Expression::ID id);
        virtual bool matches(const Expression::Ptr &expr) const;
    private:
        const Expression::ID m_id;
    };

    /**
     * @short Identifies Expression instances based on their static type.
     *
     * BySequenceTypeIdentifier identifies Expression instances
     * if their Expression::staticType() matches the requested one,
     * regardless of whether the Expression is a particular one, such
     * as AndExpression.
     *
     * For example, constructing a BySequenceTypeIdentifier while
     * passing CommonSequenceTypes::EBV in its constructor will create
     * a ExpressionIdentifier that returns @c true for a static type with
     * item type <tt>xs:string</tt>, but returns @c false for a static type involving
     * <tt>xs:date</tt>.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class BySequenceTypeIdentifier : public ExpressionIdentifier
    {
    public:
        BySequenceTypeIdentifier(const SequenceType::Ptr &seqType);

        /**
         * @returns @c true, if the static type of @p expr is matches
         * the SequenceType passed in the BySequenceTypeIdentifier()
         * constructor, otherwise @c false.
         */
        virtual bool matches(const Expression::Ptr &expr) const;

    private:
        const SequenceType::Ptr m_seqType;
    };

    /**
     * @short Determines whether an Expression is a value or general comparison or both,
     * with a certain operator.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ComparisonIdentifier : public ExpressionIdentifier
    {
    public:

        /**
         * @param comparatorHosts the possible parent that may have
         * the operator. This may be Expression::IDGeneralComparison or
         * Expression::IDValueComparison. The two values may also be OR'd,
         * meaning any of them will do.
         *
         * @param op the operator that the comparator host must have. For example,
         * if @p op is AtomicComparator::OperatorGreatorOrEqual this ComparisonIdentifier
         * will match operator >= in the case of IDGeneralComparison and 'ge' in the
         * case of IDValueComparison.
         */
        ComparisonIdentifier(const QVector<Expression::ID> comparatorHosts,
                             const AtomicComparator::Operator op);

        /**
         * @returns @c true, if @p expr is a ValueComparison with the operator
         * passed in ComparisonIdentifier().
         */
        virtual bool matches(const Expression::Ptr &expr) const;

    private:
        const QVector<Expression::ID> m_hosts;
        const AtomicComparator::Operator m_op;
    };

    /**
     * @short Matches numeric literals that are of type xs:integer and
     * has a specific value.
     *
     * For example IntegerIdentifier(4) would match the former but
     * not the latter operand in this expression: "4 + 5".
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class IntegerIdentifier : public ExpressionIdentifier
    {
    public:
        IntegerIdentifier(const xsInteger num);
        virtual bool matches(const Expression::Ptr &expr) const;

    private:
        const xsInteger m_num;
    };

    /**
     * @short Matches boolean literals.
     *
     * For example BooleanIdentifier(true) would match the former but
     * not the latter operand in this expression: "true() + false()".
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class BooleanIdentifier : public ExpressionIdentifier
    {
    public:
        BooleanIdentifier(const bool value);
        virtual bool matches(const Expression::Ptr &expr) const;

    private:
        const bool m_value;
    };

    /**
     * @short Creates a particular Expression instance identified by an Expression::ID.
     *
     * For example, if ByIDCreator() is passed Expression::IDCountFN, create()
     * will return CountFN instances.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ByIDCreator : public ExpressionCreator
    {
    public:
        /**
         * Creates a ByIDCreator that creates expressions
         * of the type that @p id identifies.
         */
        ByIDCreator(const Expression::ID id);
        virtual Expression::Ptr create(const Expression::List &operands,
                                       const StaticContext::Ptr &context,
                                       const SourceLocationReflection *const r) const;

        /**
         * Creates an expression by id @p id with the arguments @p operands.
         */
        static Expression::Ptr create(const Expression::ID id,
                                      const Expression::List &operands,
                                      const StaticContext::Ptr &context,
                                      const SourceLocationReflection *const r);

    private:
        const Expression::ID m_id;
    };
}

QT_END_NAMESPACE

#endif
