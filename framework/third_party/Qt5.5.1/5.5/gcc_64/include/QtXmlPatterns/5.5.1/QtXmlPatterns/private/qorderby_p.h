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

#ifndef Patternist_OrderBy_H
#define Patternist_OrderBy_H

#include <private/qatomiccomparator_p.h>
#include <private/qcomparisonplatform_p.h>
#include <private/qsinglecontainer_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class ReturnOrderBy;

    /**
     * @short Performs the sorting by being a parent to ForClause.
     *
     * The child of the ForClause is a ReturnOrderBy expression, which collects
     * the sort keys and values.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class OrderBy : public SingleContainer
    {
    public:
        enum Stability
        {
            StableOrder,
            UnstableOrder
        };

        /**
         * This class is value based.
         */
        class OrderSpec : public ComparisonPlatform<OrderBy::OrderSpec,
                                                    true, /* Yes, issue errors. */
                                                    AtomicComparator::AsValueComparison>
        {
        public:
            /**
             * We want this guy to be public.
             */
            using ComparisonPlatform<OrderBy::OrderSpec, true, AtomicComparator::AsValueComparison>::detailedFlexibleCompare;

            typedef QVector<OrderSpec> Vector;

            enum Direction
            {
                Ascending,
                Descending
            };

            /**
             * @short Default constructor, which is needed by QVector.
             */
            inline OrderSpec()
            {
            }

            inline OrderSpec(const Direction dir,
                             const StaticContext::OrderingEmptySequence orderingEmpty) : direction(dir),
                                                                                         orderingEmptySequence(orderingEmpty)
            {
            }

            void prepare(const Expression::Ptr &source,
                         const StaticContext::Ptr &context);

            const SourceLocationReflection *actualReflection() const
            {
                return m_expr.data();
            }

        private:
            Expression::Ptr m_expr;

        public:
            /**
             * We place these afterwards, such that m_expr gets aligned at the
             * start of the address.
             */
            Direction direction;

            StaticContext::OrderingEmptySequence orderingEmptySequence;

            inline AtomicComparator::Operator operatorID() const
            {
                return orderingEmptySequence == StaticContext::Least ? AtomicComparator::OperatorLessThanNaNLeast
                                                                     : AtomicComparator::OperatorLessThanNaNGreatest;
            }

        };

        OrderBy(const Stability stability,
                const OrderSpec::Vector &orderSpecs,
                const Expression::Ptr &operand,
                ReturnOrderBy *const returnOrderBy);

        virtual Item::Iterator::Ptr evaluateSequence(const DynamicContext::Ptr &context) const;
        virtual SequenceType::Ptr staticType() const;
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);
        virtual SequenceType::List expectedOperandTypes() const;
        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        inline Item::Iterator::Ptr mapToSequence(const Item &i,
                                                 const DynamicContext::Ptr &context) const;
        virtual Properties properties() const;

    private:
        /**
         * Needed when calling makeSequenceMappingIterator().
         */
        typedef QExplicitlySharedDataPointer<const OrderBy> ConstPtr;

        const Stability             m_stability;
        OrderSpec::Vector           m_orderSpecs;
        ReturnOrderBy *const        m_returnOrderBy;
    };

    /* TODO Q_DECLARE_TYPEINFO(OrderBy::OrderSpec, Q_MOVABLE_TYPE); Breaks,
     * probably because it's nested. */
}

QT_END_NAMESPACE

#endif
