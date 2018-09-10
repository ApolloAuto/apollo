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

#ifndef Patternist_CardinalityVerifier_H
#define Patternist_CardinalityVerifier_H

#include <private/qsinglecontainer_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Verifies that the sequence an Expression evaluates to conforms to a Cardinality.
     *
     * @see <a href="http://www.w3.org/TR/xpath-functions/#cardinality-funcs">XQuery 1.0 and
     * XPath 2.0 Functions and Operators, 15.2 Functions That Test the Cardinality of Sequences</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class CardinalityVerifier : public SingleContainer
    {
    public:
        CardinalityVerifier(const Expression::Ptr &operand,
                            const Cardinality &card,
                            const ReportContext::ErrorCode code);

        virtual Item::Iterator::Ptr evaluateSequence(const DynamicContext::Ptr &context) const;
        virtual Item evaluateSingleton(const DynamicContext::Ptr &) const;

        virtual SequenceType::List expectedOperandTypes() const;
        virtual SequenceType::Ptr staticType() const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        /**
         * If the static cardinality of the operand is within the required cardinality,
         * the operand is returned as is, since results will always be valid and hence
         * is not a CardinalityVerifier necessary.
         */
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

        /**
         * A utility function for determining whether the static type of an Expression matches
         * a cardinality. More specifically, this function performs the cardinality verification
         * part of the Function Conversion Rules.
         *
         * @todo Mention the rewrite and when exactly an error is issued via @p context
         */
        static Expression::Ptr
        verifyCardinality(const Expression::Ptr &operand,
                          const Cardinality &card,
                          const StaticContext::Ptr &context,
                          const ReportContext::ErrorCode code = ReportContext::XPTY0004);

        virtual const SourceLocationReflection *actualReflection() const;

        ID id() const;
    private:
        /**
         * Centralizes a message string in order to increase consistency and
         * reduce work for translators.
         */
        static inline QString wrongCardinality(const Cardinality &req,
                                               const Cardinality &got = Cardinality::empty());

        const Cardinality m_reqCard;
        const bool m_allowsMany;
        const ReportContext::ErrorCode m_errorCode;
    };
}

QT_END_NAMESPACE

#endif
