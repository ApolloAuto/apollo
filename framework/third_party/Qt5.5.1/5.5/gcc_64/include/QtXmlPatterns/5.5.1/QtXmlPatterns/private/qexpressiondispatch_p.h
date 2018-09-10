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

#ifndef Patternist_ExpressionDispatch_H
#define Patternist_ExpressionDispatch_H

#include <QSharedData>


QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class AndExpression;
    class ApplyTemplate;
    class ArgumentConverter;
    class ArgumentReference;
    class ArithmeticExpression;
    class Atomizer;
    class AttributeConstructor;
    class AttributeNameValidator;
    class AxisStep;
    class CallTemplate;
    class CardinalityVerifier;
    class CardinalityVerifier;
    class CastableAs;
    class CastableAs;
    class CastAs;
    class CastAs;
    class CollationChecker;
    class CollationChecker;
    class CombineNodes;
    class CombineNodes;
    class CommentConstructor;
    class CommentConstructor;
    class ComputedNamespaceConstructor;
    class ContextItem;
    class CopyOf;
    class CurrentItemStore;
    class DocumentConstructor;
    class DynamicContextStore;
    class EBVExtractor;
    class ElementConstructor;
    class EmptySequence;
    class ExpressionSequence;
    class ExpressionVariableReference;
    class ExternalVariableReference;
    class FirstItemPredicate;
    class ForClause;
    class FunctionCall;
    class GeneralComparison;
    class GenericPredicate;
    class IfThenClause;
    class InstanceOf;
    class ItemVerifier;
    class LetClause;
    class Literal;
    class LiteralSequence;
    class NamespaceConstructor;
    class NCNameConstructor;
    class NodeComparison;
    class NodeSortExpression;
    class OrderBy;
    class OrExpression;
    class ParentNodeAxis;
    class Path;
    class PositionalVariableReference;
    class ProcessingInstructionConstructor;
    class QNameConstructor;
    class QuantifiedExpression;
    class RangeExpression;
    class RangeVariableReference;
    class ReturnOrderBy;
    class SimpleContentConstructor;
    class StaticBaseURIStore;
    class StaticCompatibilityStore;
    class TemplateParameterReference;
    class TextNodeConstructor;
    class TreatAs;
    class TruthPredicate;
    class UnresolvedVariableReference;
    class UntypedAtomicConverter;
    class UserFunctionCallsite;
    class ValidationError;
    class ValueComparison;
    template<bool IsForGlobal> class EvaluationCache;

    /**
     * @todo Documentation's missing
     *
     * @defgroup Patternist_expr_dispatch Expression Dispatching
     */

    /**
     * @ingroup Patternist_expr_dispatch
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ExpressionVisitorResult : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ExpressionVisitorResult> Ptr;
        ExpressionVisitorResult() {}
        virtual ~ExpressionVisitorResult() {}
    };

    /**
     * @ingroup Patternist_expr_dispatch
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ExpressionVisitor : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ExpressionVisitor> Ptr;
        virtual ~ExpressionVisitor() {}

        virtual ExpressionVisitorResult::Ptr visit(const AndExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ApplyTemplate *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ArgumentConverter *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ArgumentReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ArithmeticExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const Atomizer *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const AttributeConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const AttributeNameValidator *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const AxisStep *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CallTemplate *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CardinalityVerifier *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CastableAs *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CastAs *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CollationChecker *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CombineNodes *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CommentConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ComputedNamespaceConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ContextItem *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CopyOf *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const CurrentItemStore *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const DocumentConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const DynamicContextStore *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const EBVExtractor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ElementConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const EmptySequence *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const EvaluationCache<false> *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const EvaluationCache<true> *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ExpressionSequence *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ExpressionVariableReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ExternalVariableReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const FirstItemPredicate *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ForClause *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const FunctionCall *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const GeneralComparison *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const GenericPredicate *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const IfThenClause *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const InstanceOf *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ItemVerifier *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const LetClause *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const Literal *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const LiteralSequence *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const NamespaceConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const NCNameConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const NodeComparison *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const NodeSortExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const OrderBy *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const OrExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ParentNodeAxis *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const Path *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const PositionalVariableReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ProcessingInstructionConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const QNameConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const QuantifiedExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const RangeExpression *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const RangeVariableReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ReturnOrderBy *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const SimpleContentConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const StaticBaseURIStore *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const StaticCompatibilityStore *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const TemplateParameterReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const TextNodeConstructor *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const TreatAs *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const TruthPredicate *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const UnresolvedVariableReference *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const UntypedAtomicConverter *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const UserFunctionCallsite *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ValidationError *) const = 0;
        virtual ExpressionVisitorResult::Ptr visit(const ValueComparison *) const = 0;
    };
}

QT_END_NAMESPACE

#endif
