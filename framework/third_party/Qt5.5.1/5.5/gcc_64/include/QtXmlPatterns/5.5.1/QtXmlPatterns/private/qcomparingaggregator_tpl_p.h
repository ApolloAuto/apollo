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
//

/**
 * @file qcomparingaggregator.cpp
 * @short This file is included by qcomparingaggregator_p.h.
 * If you need includes in this file, put them in qcomparingaggregator_p.h, outside of the namespace.
 */

template <AtomicComparator::Operator oper, AtomicComparator::ComparisonResult result>
inline Item
ComparingAggregator<oper, result>::applyNumericPromotion(const Item &old,
                                                         const Item &nev,
                                                         const Item &newVal) const
{
    Q_ASSERT(old);
    Q_ASSERT(nev);
    Q_ASSERT(newVal);
    const ItemType::Ptr to(old.type());
    const ItemType::Ptr tn(nev.type());

    if(!(BuiltinTypes::numeric->xdtTypeMatches(to) && BuiltinTypes::numeric->xdtTypeMatches(tn)))
        return newVal; /* At least one of them isn't numeric. */
    else if(BuiltinTypes::xsDouble->xdtTypeMatches(to) || BuiltinTypes::xsDouble->xdtTypeMatches(tn))
        return toItem(Double::fromValue(newVal.as<Numeric>()->toDouble()));
    else if(BuiltinTypes::xsFloat->xdtTypeMatches(to) || BuiltinTypes::xsFloat->xdtTypeMatches(tn))
        return toItem(Float::fromValue(newVal.as<Numeric>()->toDouble()));
    else if(BuiltinTypes::xsInteger->xdtTypeMatches(to) &&
            BuiltinTypes::xsInteger->xdtTypeMatches(tn))
        return newVal; /* Both must be xs:integer. */
    else
        return toItem(Decimal::fromValue(newVal.as<Numeric>()->toDecimal()));
}

template <AtomicComparator::Operator oper, AtomicComparator::ComparisonResult result>
Item
ComparingAggregator<oper, result>::evaluateSingleton(const DynamicContext::Ptr &context) const
{
    const Item::Iterator::Ptr it(m_operands.first()->evaluateSequence(context));
    Item largest;

    while(true)
    {
        Item next(it->next());

        if(!next)
        {
            return largest;
        }

        AtomicComparator::Ptr comp(comparator());

        if(!comp)
        {
            ItemType::Ptr t1(next.type());
            Q_ASSERT(t1);

            if(BuiltinTypes::xsUntypedAtomic->xdtTypeMatches(t1))
            {
                next = cast(next, context);
                t1 = BuiltinTypes::xsDouble;
            }

            if(!largest)
            {
                largest = next;
                continue;
            }

            Q_ASSERT(largest);
            comp = fetchComparator(largest.type(), t1, context);
            Q_ASSERT(comp);
        }
        else if(!largest)
        {
            largest = next;
            continue;
        }

        if(comp->compare(next, operatorID(), largest) == result)
        {
            largest = applyNumericPromotion(largest, next, next);
            continue;
        }

        const ItemType::Ptr t(next.type());

        if(BuiltinTypes::xsDouble->xdtTypeMatches(t) &&
           next.as<Numeric>()->isNaN())
        {
            return CommonValues::DoubleNaN;
        }
        else if(BuiltinTypes::xsFloat->xdtTypeMatches(t) &&
                next.as<Numeric>()->isNaN())
        {
            if(BuiltinTypes::xsDouble->xdtTypeMatches(largest.type()))
                return CommonValues::DoubleNaN;

            /* If we have a xs:double somewhere, we must promote the NaN value to xs:double,
             * and we really should raise error on invalid value. */
            largest = it->next();

            while(largest)
            {
                const ItemType::Ptr tf(largest.type());
                if(BuiltinTypes::xsDouble->xdtTypeMatches(tf))
                    return CommonValues::DoubleNaN;
                else if(BuiltinTypes::xsUntypedAtomic->xdtTypeMatches(tf))
                {
                    /* Attempt a convert, which will raise an error if it doesn't work out. */
                    cast(largest, context);
                    return CommonValues::DoubleNaN;
                }
                else if(!BuiltinTypes::numeric->xdtTypeMatches(tf))
                {
                    fetchComparator(BuiltinTypes::xsFloat, tf, context);
                }
                else
                    largest = it->next();
            };

            return CommonValues::FloatNaN;
        }
        else
            largest = applyNumericPromotion(largest, next, largest);
    }
}

template <AtomicComparator::Operator oper, AtomicComparator::ComparisonResult result>
Expression::Ptr
ComparingAggregator<oper, result>::typeCheck(const StaticContext::Ptr &context,
                                             const SequenceType::Ptr &reqType)
{
    Q_ASSERT(oper == AtomicComparator::OperatorGreaterThan ||
             oper == AtomicComparator::OperatorLessThan);
    const Expression::Ptr me(FunctionCall::typeCheck(context, reqType));

    ItemType::Ptr t1(m_operands.first()->staticType()->itemType());

    if(*CommonSequenceTypes::Empty == *t1)
        return EmptySequence::create(this, context);
    else if(*BuiltinTypes::xsAnyAtomicType == *t1 ||
            BuiltinTypes::numeric->xdtTypeMatches(t1))
        return me;
    else if(BuiltinTypes::xsUntypedAtomic->xdtTypeMatches(t1))
    {
        m_operands.replace(0, Expression::Ptr(new UntypedAtomicConverter(m_operands.first(),
                           BuiltinTypes::xsDouble)));
        t1 = m_operands.first()->staticType()->itemType();
    }
    else if(!BuiltinTypes::xsString->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsAnyURI->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsDayTimeDuration->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsDate->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsTime->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsDateTime->xdtTypeMatches(t1) &&
            !BuiltinTypes::xsYearMonthDuration->xdtTypeMatches(t1))
    {
        context->error(QtXmlPatterns::tr("The first argument to %1 cannot be of type %2.")
                          .arg(QPatternist::formatFunction(context->namePool(), signature()))
                          .arg(formatType(context->namePool(), m_operands.first()->staticType())),
                       ReportContext::FORG0006, this);
        return me;
    }

    if(!m_operands.first()->staticType()->cardinality().allowsMany())
        return m_operands.first();

    // explicit scope needed in RVCT
    ComparingAggregator<oper, result>::prepareComparison(fetchComparator(t1, t1, context));

    return me;
}

