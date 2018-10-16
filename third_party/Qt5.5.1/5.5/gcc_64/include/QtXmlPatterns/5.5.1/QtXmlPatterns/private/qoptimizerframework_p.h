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

#ifndef Patternist_OptimizerFramework_H
#define Patternist_OptimizerFramework_H

#include <QSharedData>

#include <private/qexpression_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A factory for creating Expression instances.
     *
     * ExpressionIdentifier is one of the building block of Patternist's
     * optimizer framework. An ExpressionIdentifier sub-class has
     * the responsibility of creating the Expression that should be
     * the result of the optimization.
     *
     * This class and sub-classes are never used on their own,
     * but in cooperation with OptimizationPass.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ExpressionCreator : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ExpressionCreator> Ptr;

        /**
         * For some reason this constructor cannot be synthesized.
         */
        inline ExpressionCreator()
        {
        }

        virtual ~ExpressionCreator();
        /**
         * Creates an expression that has @p operands as operands.
         *
         * The Expression that is returned is guaranteed, by the caller,
         * to get a treatment identical to if the expression was created
         * in an ordinary compilation(via the parser, and so forth). That is,
         * Expression::typeCheck() and Expression::compress() stages will be
         * carried out on the returned expression.
         *
         * @returns an Expression::Ptr that never is non @c null, valid pointer
         */
        virtual Expression::Ptr create(const Expression::List &operands,
                                       const StaticContext::Ptr &context,
                                       const SourceLocationReflection *const) const = 0;

    private:
        Q_DISABLE_COPY(ExpressionCreator)
    };

    /**
     * @short Abstract base class for all classes that identify Expressions
     * based on some criteria.
     *
     * ExpressionIdentifier is one of the building block of Patternist's
     * optimizer framework. An ExpressionIdentifier sub-class has
     * the responsibility of determining whether a particular Expression
     * is the one an OptimizationPass should apply for.
     *
     * This class and sub-classes are never used on their own,
     * but in cooperation with OptimizationPass.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ExpressionIdentifier : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ExpressionIdentifier> Ptr;
        typedef QList<ExpressionIdentifier::Ptr> List;

        /**
         * For some reason this constructor cannot be synthesized.
         */
        inline ExpressionIdentifier()
        {
        }

        virtual ~ExpressionIdentifier();
        /**
         * @param expr the Expression to be tested. This is guranteed
         * to always be a non @c null, valid pointer.
         *
         * @returns @c true if @p expr matches as according to this ExpressionIdentifier,
         * otherwise @c false.
         */
        virtual bool matches(const Expression::Ptr &expr) const = 0;

    private:
        Q_DISABLE_COPY(ExpressionIdentifier)
    };

    /**
     * @short Describes how a particular optimization pass should be carried out.
     *
     * OptimizationPass is essentially a declaration, which describes
     * how an optimization pass in the form of an AST rewrite should be done,
     * by describing what that should be rewritten into what how.
     *
     * Each OptimizationPass is applied to a "start" Expression. The Expression
     * that qualifies as a start Expression for the OptimizationPass in question is
     * determined by startIdentifier; if its ExpressionIdentifier::matches() function
     * returns @c true, the optimizer continues to apply this OptimizationPass.
     *
     * After a start Expression has been found, it is verified if the operands matches
     * as well by applying the ExpressionIdentifiers in operandIdentifiers to the
     * start Expression's operands. Similarly, if the operands matches what
     * operandIdentifiers requires, the optimizer continues to apply this OptimizationPass.
     *
     * At this stage, it has been concluded that the OptimizationPass validly applies, and
     * what now remains is to carry out the actual rewrite. The Expression rewritten
     * to is the one returned by ExpressionCreator::create(), when invoked via the resultCreator
     * variable.
     *
     * How these components, startIdentifier, operandIdentifiers, sourceExpression,
     * and resultCreator interacts with one another is described in more detail
     * in the member documentation as well as the classes they are instances of.
     *
     * @author Frans englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class OptimizationPass : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<OptimizationPass> Ptr;
        typedef QList<OptimizationPass::Ptr> List;

        enum OperandsMatchMethod
        {
            /**
             * All operands must match in the same order the ExpressionMarkers do.
             */
            Sequential = 1,

            /**
             * Matches if all operands are matched, regardless of their order. This is
             * useful when an OptimizationPass is matching an Expression that has two operands
             * and that both of them can appear on the left or right hand as long as it is those
             * two.
             *
             * This comparison method only works when two operands
             * needs to be matched.
             */
            AnyOrder
        };

        /**
         * An ExpressionMarker identifies an operand Expression relatively
         * the start Expression by that each integer identifies a step
         * in a descending AST walk. For example an ExpressionMarker with
         * only one entry that is 0(zero), identifies the first operand of the
         * start Expression. An ExpressionMarker containing 1, 2 in that order
         * identifies the third operand of the second operand of the start Expression.
         */
        typedef QList<qint8> ExpressionMarker;

        /**
         * Creates an OptimizationPass and sets its public variables
         * to the corresponding values passed in this constructor.
         */
        OptimizationPass(const ExpressionIdentifier::Ptr &startID,
                         const ExpressionIdentifier::List &operandIDs,
                         const ExpressionMarker &sourceExpr,
                         const ExpressionCreator::Ptr &resultCtor = ExpressionCreator::Ptr(),
                         const OperandsMatchMethod matchMethod = Sequential);

        /**
         * The ExpressionIdentifier that must the Expression this OptimizationPass concerns.
         *
         * If this variable is @c null, it means that the start Expression does
         * not have to match any particular ExpressionIdentifier, but is fine as is.
         *
         * One might wonder what the purpose of this startIdentifier is, considering
         * that what start Expression an OptimizationPass can at all apply to is
         * naturally determined by what Expression::optimizationPasses() re-implementation that
         * returns this OptimizationPass. The reason is that in some cases an OptimizationPass
         * nevertheless doesn't apply. For example, optimizations applying to a ValueComparison
         * might depend on what operator that is in use.
         *
         * May be @c null or point to an ExpressionIdentifier.
         */
        const ExpressionIdentifier::Ptr startIdentifier;

        /**
         * In order for an OptimizationPass to apply, the start Expression's
         * operands must be matched with this list of ExpressionIdentifier instances.
         * The first ExpressionIdentifier is applied to the first operand, the second
         * ExpressionIdentifier to the second operand, and so forth until all operands
         * have been iterated.
         *
         * Entries in this list may be @c null, and those signals that the corresponding
         * operand is not constrained. For example, if the third ExpressionIdentifier in
         * the list is @c null, it signals that the third operand may be anykind of expression.
         *
         * May be empty or contain an arbitrary amount of objects or @c null pointers.
         */
        const ExpressionIdentifier::List operandIdentifiers;

        /**
         * Identifies the expression that should be part of the new expression
         * that this OptimizationPass rewrites to. If this list is empty, it
         * means that the result is not derived from the existing tree, and
         * that resultCreator will exclusively be used for creating the result
         * Expression.
         *
         * How the ExpressionMarker identifies an Expression is document in
         * its documentation.
         *
         * May be empty.
         */
        const ExpressionMarker sourceExpression;

        /**
         * This is the ExpressionCreator that will be used to create the
         * Expression which is the result. ExpressionCreator::create()
         * will be passed as operands the Expression that sourceExpression
         * specify, if any.
         *
         * If this variable is @c null, the result Expression will be the one
         * sourceExpression identifies.
         */
        const ExpressionCreator::Ptr resultCreator;

        const OperandsMatchMethod operandsMatchMethod;
    private:
        Q_DISABLE_COPY(OptimizationPass)
    };
}

QT_END_NAMESPACE

#endif
