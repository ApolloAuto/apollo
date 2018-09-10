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

#ifndef Patternist_TruthPredicate_H
#define Patternist_TruthPredicate_H

#include <private/qgenericpredicate_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A predicate which is optimized for filter expressions that
     * are of type @c xs:boolean.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class TruthPredicate : public GenericPredicate
    {
    public:
        /**
         * Creates a TruthPredicate which filters the items from the @p sourceExpression
         * through @p predicate.
         *
         * This constructor is protected. The proper way to create predicates is via the static
         * create() function.
         */
        TruthPredicate(const Expression::Ptr &sourceExpression,
                       const Expression::Ptr &predicate);

        inline Item mapToItem(const Item &item, const DynamicContext::Ptr &context) const
        {
            Q_ASSERT_X(false, Q_FUNC_INFO, "This is practically dead code because it never gets called in GenericPredicate, "
                                           "which binds to its own mapToItem for completely legitime reasons.");
            if(m_operand2->evaluateEBV(context))
                return item;
            else
                return Item();
        }

        inline Item::Iterator::Ptr map(const Item &item,
                                       const DynamicContext::Ptr &context) const
        {
            Q_ASSERT_X(false, Q_FUNC_INFO, "I don't expect this function to be called, for the same reasons as above.");
            if(m_operand2->evaluateEBV(context))
                return makeSingletonIterator(item);
            else
                return CommonValues::emptyIterator;
        }

        virtual SequenceType::List expectedOperandTypes() const;
        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;
    };
}

QT_END_NAMESPACE

#endif
