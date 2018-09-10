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

#ifndef Patternist_GeneralComparison_H
#define Patternist_GeneralComparison_H

#include <private/qatomiccomparator_p.h>
#include <private/qpaircontainer_p.h>
#include <private/qcomparisonplatform_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Implements XPath 2.0's general comparions, such as the <tt>=</tt> operator.
     *
     * ComparisonPlatform is inherited with @c protected scope because ComparisonPlatform
     * must access members of GeneralComparison.
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-general-comparisons">XML Path Language
     * (XPath) 2.0, 3.5.2 General Comparisons</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class GeneralComparison : public PairContainer,
                              public ComparisonPlatform<GeneralComparison,
                                                        true /* We want to report errors. */,
                                                        AtomicComparator::AsGeneralComparison>
    {
    public:
        GeneralComparison(const Expression::Ptr &op1,
                          const AtomicComparator::Operator op,
                          const Expression::Ptr &op2,
                          const bool isBackwardsCompat = false);

        virtual bool evaluateEBV(const DynamicContext::Ptr &) const;
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        virtual SequenceType::List expectedOperandTypes() const;
        virtual SequenceType::Ptr staticType() const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        /**
         * @returns always IDGeneralComparison
         */
        virtual ID id() const;

        virtual QList<QExplicitlySharedDataPointer<OptimizationPass> > optimizationPasses() const;

        /**
         * @returns the operator that this GeneralComparison is using.
         */
        inline AtomicComparator::Operator operatorID() const
        {
            return m_operator;
        }

        /**
         * Overridden to optimize case-insensitive compares.
         */
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

    private:
        static inline void updateType(ItemType::Ptr &type,
                                      const Expression::Ptr &source);
        AtomicComparator::Ptr fetchGeneralComparator(Expression::Ptr &op1,
                                                     Expression::Ptr &op2,
                                                     const ReportContext::Ptr &context) const;
        bool generalCompare(const Item &op1,
                            const Item &op2,
                            const DynamicContext::Ptr &context) const;

        const AtomicComparator::Operator m_operator;
        const bool m_isBackwardsCompat;
    };
}

QT_END_NAMESPACE

#endif
