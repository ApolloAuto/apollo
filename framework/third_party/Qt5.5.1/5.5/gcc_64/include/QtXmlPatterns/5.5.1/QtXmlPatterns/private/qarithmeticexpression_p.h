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

#ifndef Patternist_ArithmeticExpression_H
#define Patternist_ArithmeticExpression_H

#include <private/qatomicmathematician_p.h>
#include <private/qpaircontainer_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Implements arithmetics, such as multiplication and subtraction.
     *
     *
     * Optimizations: there's some operator/value combos that are no ops. For
     * instance, 0 + <value>, which is the case of unary plus. We can't compile
     * those away early due to that type checks needs to be done but one can
     * check for them in compress().
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-arithmetic">XML Path Language
     * (XPath) 2.0, 3.4 Arithmetic Expressions</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class Q_AUTOTEST_EXPORT ArithmeticExpression : public PairContainer
    {
    public:
        ArithmeticExpression(const Expression::Ptr &operand1,
                             const AtomicMathematician::Operator op,
                             const Expression::Ptr &operand2);

        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;

        virtual SequenceType::Ptr staticType() const;
        virtual SequenceType::List expectedOperandTypes() const;
        AtomicMathematician::Operator operatorID() const;

        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        static Item flexiblyCalculate(const Item &op1,
                                      const AtomicMathematician::Operator op,
                                      const Item &op2,
                                      const AtomicMathematician::Ptr &mather,
                                      const DynamicContext::Ptr &context,
                                      const SourceLocationReflection *const reflection,
                                      const ReportContext::ErrorCode code = ReportContext::XPTY0004,
                                      const bool isCompat = false);

        static AtomicMathematician::Ptr
        fetchMathematician(Expression::Ptr &t1,
                           Expression::Ptr &t2,
                           const AtomicMathematician::Operator op,
                           const bool issueError,
                           const ReportContext::Ptr &context,
                           const SourceLocationReflection *const reflection,
                           const ReportContext::ErrorCode code = ReportContext::XPTY0004,
                           const bool isCompat = false);
        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

    protected:

    private:
        const AtomicMathematician::Operator m_op;
        AtomicMathematician::Ptr            m_mather;
        bool                                m_isCompat;
    };

    inline AtomicMathematician::Operator ArithmeticExpression::operatorID() const
    {
        return m_op;
    }

}

QT_END_NAMESPACE

#endif
