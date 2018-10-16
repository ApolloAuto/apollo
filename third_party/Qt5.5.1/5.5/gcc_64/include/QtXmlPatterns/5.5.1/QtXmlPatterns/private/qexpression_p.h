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

#ifndef Patternist_Expression_H
#define Patternist_Expression_H

#include <QFlags>
#include <QSharedData>

#include <private/qcppcastinghelper_p.h>
#include <private/qxmldebug_p.h>
#include <private/qdynamiccontext_p.h>
#include <private/qexpressiondispatch_p.h>
#include <private/qitem_p.h>
#include <private/qsequencetype_p.h>
#include <private/qsourcelocationreflection_p.h>
#include <private/qstaticcontext_p.h>

QT_BEGIN_NAMESPACE

template<typename T> class QList;
template<typename T> class QVector;

namespace QPatternist
{
    template<typename T, typename ListType> class ListIterator;
    class OptimizationPass;

    /**
     * @short Base class for all AST nodes in an XPath/XQuery/XSL-T expression.
     *
     * @section ExpressionCreation Expression Compilation
     *
     * @subsection ExpressionCreationParser The process of creating an Expression
     *
     * The initial step of creating an internal representation(in some circles
     * called an IR tree) of the XPath string follows classic compiler design: a scanner
     * is invoked, resulting in tokens, which sub-sequently are consumed by a parser
     * which groups the tokens into rules, resulting in the creation of
     * Abstract Syntax Tree(AST) nodes that are arranged in a hierarchical structure
     * similar to the EBNF.
     *
     * More specifically, ExpressionFactory::createExpression() is called with a
     * pointer to a static context, and the string for the expression. This is subsequently
     * tokenized by a Flex scanner. Mistakes detected at this stage is syntax
     * errors, as well as a few semantical errors. Syntax errors can be divided
     * in two types:
     *
     *     - The scanner detects it. An example is the expression "23Eb3" which
     *     is not a valid number literal, or "1prefix:my-element" which is not a
     *     valid QName.
     *     - The parser detects it. This means a syntax error at a
     *     higher level, that a group of tokens couldn't be reduced to a
     *     rule(expression). An example is the expression "if(a = b) 'match' else
     *     'no match'"; the tokenizer would handle it fine, but the parser would
     *     fail because the tokens could not be reduced to a rule due to the token
     *     for the "then" word was missing.
     *
     * Apart from the syntax errors, the actions in the parser also detects
     * errors when creating the corresponding expressions. This is for example
     * that no namespace binding for a prefix could be found, or that a function
     * call was used which no function implementation could be found for.
     *
     * When the parser has finished, the result is an AST. That is, a
     * hierarchical structure consisting of Expression sub-classes. The
     * individual expressions haven't at this point done anything beyond
     * receiving their child-expressions(if any), and hence reminds of a
     * "construction scaffold". In other words, a tree for the expression
     * <tt>'string' + 1 and xs:date('2001-03-13')</tt> could have been created, even if
     * that expression contains errors(one can't add a xs:integer to a xs:string,
     * and the Effective %Boolean Value cannot be extracted for date types).
     *
     * @subsection ExpressionCreationTypeChecking Type Checking
     *
     * After the AST creation, ExpressionFactory::createExpression continues with
     * calling the AST node(which is an Expression instance)'s typeCheck()
     * function. This step ensures that the static types of the operands matches
     * the operators, and in the cases where it doesn't, modifies the AST such
     * that the necessary conversions are done -- if possible, otherwise the
     * result is a type error.
     *
     *
     * This step corresponds roughly to what <a
     * href="http://www.w3.org/TR/xpath20/#id-static-analysis">2.2.3.1 Static Analysis Phase</a>
     * labels operation tree normalization; step SQ5.
     *
     * @subsection ExpressionCreationCompression Compressing -- Optimization and Fixup
     *
     * The last step is calling compress(). This function is not called
     * 'optimize', 'simplify' or the like, because although it performs all
     * optimization, it also involves mandatory stages.
     *
     * One such is const folding, which while being an efficient optimization,
     * also is a necessity for many XSL-T constructs. Another important step is
     * that functions which had an evaluation dependency on the static context(as
     * opposed to the dynamic) performs their "fixup".
     *
     * In other words, this stage potentially performs AST re-writes. For example,
     * the expression <tt>3 + 3, concat('foo', '-', 'bar'), true() and false()</tt> would
     * result in an AST corresponding to <tt>6, 'foo-bar', false()</tt>. This process
     * is done backwards; each expression asks its operands to compress before it
     * performs its own compression(and so forth, until the root expression's call
     * returns to the caller).
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-errors-and-opt">XML Path Language
     * (XPath) 2.0, 2.3.4 Errors and Optimization</a>
     * @see <a href="http://www.w3.org/TR/xpath20/#id-expression-processing">XML Path
     * Language (XPath) 2.0, 2.2.3 Expression Processing</a>
     * @see <a href="http://www.w3.org/TR/xquery-xpath-parsing/">Building a Tokenizer
     * for XPath or XQuery</a>
     * @see ExpressionFactory
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class Q_AUTOTEST_EXPORT Expression : public QSharedData
                                       , public CppCastingHelper<Expression>
                                       , public SourceLocationReflection
    {
    public:
        /**
         * @short A smart pointer wrapping mutable Expression instances.
         */
        typedef QExplicitlySharedDataPointer<Expression> Ptr;

        /**
         * @short A smart pointer wrapping @c const Expression instances.
         */
        typedef QExplicitlySharedDataPointer<const Expression> ConstPtr;

        /**
         * A list of Expression instances, each wrapped in a smart pointer.
         */
        typedef QList<Expression::Ptr> List;

        /**
         * A vector of Expression instances, each wrapped in a smart pointer.
         */
        typedef QVector<Expression::Ptr> Vector;

        typedef QT_PREPEND_NAMESPACE(QAbstractXmlForwardIterator<Expression::Ptr>)
            QAbstractXmlForwardIterator;

        /**
         * Enum flags describing the characteristics of the expression.
         *
         * @see Expression::properties()
         */
        enum Property
        {
            /**
             * This flag applies for functions, and results in the expression <tt>.</tt>
             * being appended to its operands if its operand count is lower than the
             * maximum amount of arguments.
             *
             * In effect, it result in a modification of the function's arguments to have
             * appended the context item.
             *
             * One function which has this property is <tt>fn:number()</tt>.
             *
             * @see ContextItem
             * @see <a href="http://www.w3.org/TR/xpath-functions/#func-signatures">XQuery 1.0 and
             * XPath 2.0 Functions and Operators, 1.3 Function Signatures and Descriptions</a>
             */
            UseContextItem              = 1,

            /**
             * Disables compression(evaluation at compile time), such that the
             * Expression isn't const-folded, but ensured to be run at runtime. The
             * operands are still attempted to be compressed, unless
             * they override compression as well.
             *
             * @see compress()
             */
            DisableElimination          = 1 << 1,

            /**
             * Signals that the expression is already evaluated and can be considered
             * a constant value.
             * For example, atomic values return this flag in their
             * implementations of the properties() functions.
             *
             * @see isEvaluated()
             */
            IsEvaluated                 = 1 << 2,

            /**
             * Signals that the expression cannot be optimized away by judging
             * its static type.
             *
             * This is currently used for properly handling the @c none type, in
             * the <tt>fn:error()</tt> function. In type operations, the none type doesn't show
             * up and that can make expressions, such as InstanceOf, believe
             * it is safe to const fold, while it in fact is not.
             */
            DisableTypingDeduction      = 1 << 3,

            /**
             * This property affects the static type -- staticType() -- of an expression. It
             * is implemented in FunctionCall::staticType() and therefore only work for FunctionCall
             * sub-classes and when that function is not re-implemented in an inhibiting way.
             *
             * When set, the cardinality of the static type is zero if the Expression's first
             * operand allows an empty sequence, otherwise it is the cardinality of the Expression's
             * static type modulo Cardinality::empty(). This is used for specifying proper static
             * type inference for functions that have "If $arg is the empty sequence,
             * the empty sequence is returned." However, before setting this property one
             * must be aware that no other conditions can lead to the empty sequence, since
             * otherwise the static type would be wrong.
             */
            EmptynessFollowsChild       = 1 << 4,

            /**
             * This is similar to EmptynessFollowsChild, and also implemented in FunctionCall.
             * When set, it makes FunctionCall::typeCheck() rewrite itself into an empty sequence
             * if the first operand is the empty sequence.
             *
             * This property is often used together with EmptynessFollowsChild.
             */
            RewriteToEmptyOnEmpty       = 1 << 5,

            /**
             * When set, it signals that the focus cannot be undefined. For example,
             * the <tt>fn:position()</tt> function extracts information from the focus. Setting
             * this flag ensures type checking is carried out appropriately.
             *
             * However, setting RequiresFocus does not imply this Expression requires the context
             * item to be defined. It only means the focus, of somekind, needs to be defined.
             *
             * @see RequiresContextItem
             */
            RequiresFocus               = 1 << 6,

            /**
             * An Expression with this Property set, signals that it only affects
             * the order of its return value.
             */
            AffectsOrderOnly            = 1 << 7,

            /**
             * When set, signals that the context item, must be defined for this Expression. When
             * setting this property, expectedContextItemType() must be re-implemented.
             *
             * Setting this property also sets RequiresFocus.
             *
             * @see DynamicContext::contextItem()
             */
            RequiresContextItem         = (1 << 8) | RequiresFocus,

            /**
             * When set, signals that this expression creates a focus for its last operand.
             * When set, newFocusType() must be overridden to return the static type
             * of the context item.
             *
             * @see announceFocusType()
             * @see newFocusType()
             */
            CreatesFocusForLast         = 1 << 9,

            /**
             * Signals that the last operand is a collation argument. This ensures
             * that the necessary code is generated for checking that the collation
             * is supported.
             *
             * This only applies to sub-classes of FunctionCall.
             */
            LastOperandIsCollation      = 1 << 10,

            /**
             * When set, the Expression depends on local variables such as
             * those found in @c for expressions. However, this does not
             * include let bindings.
             */
            DependsOnLocalVariable      = (1 << 11) | DisableElimination,

            /**
             * When set, it signals that the Expression does not need
             * an evaluation cache, despite what other flags might imply.
             */
            EvaluationCacheRedundant       = (1 << 12),

            /**
             * Signals that the Expression constructs nodes, either directly
             * or computationally. For example, AttributeConstructor has this property
             * set.
             *
             * Since node constructors constructs nodes which have node
             * identities, node constructors are considered creative on
             * evaluation.
             */
            IsNodeConstructor           = 1 << 13,

            /**
             * Whether this expression requires the current item, as returned
             * from @c fn:current().
             *
             * CurrentFN uses this flag.
             */
            RequiresCurrentItem         = 1 << 14
        };

        /**
         * A QFlags template for type-safe handling of ExpressionProperty values. If
         * Expression::Property flags needs to be stored in a class, declared the variable
         * to be of type Expression::Properties.
         *
         * @see QFlags
         */
        typedef QFlags<Property> Properties;

        /**
         * Enumerators that identifies Expression sub-classes.
         *
         * @see id()
         */
        enum ID
        {
            /**
             * Identifies Boolean.
             */
            IDBooleanValue = 1,

            /**
             * Identifies CountFN.
             */
            IDCountFN,

            /**
             * Identifies EmptyFN.
             */
            IDEmptyFN,

            /**
             * Identifies ExistsFN.
             */
            IDExistsFN,

            /**
             * Identifies ExpressionSequence.
             */
            IDExpressionSequence,

            /**
             * Identifies LiteralSequence.
             */
            IDLiteralSequence,

            /**
             * Identifies GeneralComparison.
             */
            IDGeneralComparison,

            /**
             * Identifies IfThenClause.
             */
            IDIfThenClause,

            /**
             * Identifies nothing in particular. The default implementation
             * of id() returns this, which is suitable for Expression instances
             * which never needs to be identified in this aspect.
             */
            IDIgnorableExpression,

            /**
             * Identifies Integer.
             */
            IDIntegerValue,

            /**
             * Identifies PositionFN.
             */
            IDPositionFN,

            /**
             * Identifies AtomicString, AnyURI, and UntypedAtomic.
             */
            IDStringValue,

            /**
             * Identifies ValueComparison.
             */
            IDValueComparison,

            /**
             * Identifies VariableReference.
             */
            IDRangeVariableReference,

            /**
             * Identifies ContextItem.
             */
            IDContextItem,

            /**
             * Identifies UserFunctionCallsite.
             */
            IDUserFunctionCallsite,

            /**
             * Identifies ExpressionVariableReference.
             */
            IDExpressionVariableReference,

            /**
             * Identifies ExpressionVariableReference.
             */
            IDAttributeConstructor,

            /**
             * Identifies UpperCaseFN.
             */
            IDUpperCaseFN,

            /**
             * Identifies LowerCaseFN.
             */
            IDLowerCaseFN,

            /**
             * Identifies FirstItemPredicate.
             */
            IDFirstItemPredicate,
            IDEmptySequence,
            IDReturnOrderBy,
            IDLetClause,
            IDForClause,
            IDPath,
            IDNamespaceConstructor,
            IDArgumentReference,
            IDGenericPredicate,
            IDAxisStep,

            /**
             * A literal which is either @c xs:float or
             * @c xs:double.
             */
            IDFloat,

            IDCombineNodes,
            IDUnresolvedVariableReference,
            IDCardinalityVerifier
        };

        inline Expression()
        {
        }
        virtual ~Expression();

        /**
         * Evaluate this Expression by iterating over it. This is a central function
         * for evaluating expressions.
         *
         * Expressions must always always return a valid QAbstractXmlForwardIterator and may
         * never return 0. If an empty result is of interest to be returned, the
         * EmptyIterator should be returned.
         *
         * The default implementation returns a SingletonIterator over the
         * item returned from evaluateSingleton().
         *
         * @note This function may raise an exception when calling, not only
         * when QAbstractXmlForwardIterator::next() is called on the return value. This is because
         * in some cases evaluateSingleton() is called directly.
         */
        virtual Item::Iterator::Ptr evaluateSequence(const DynamicContext::Ptr &context) const;

        /**
         * @todo Docs
         */
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;

        /**
         * Determines the Effective %Boolean Value of the expression.
         *
         * The Effective %Boolean Value of a value is not necessarily the same
         * as converting the value to a new value of type xs:boolean.
         *
         * Note that this function cannot return the empty sequence,
         * evaluateSingleton() must be overridden in order to be able to do
         * that.
         *
         * The default implementation results in a type error. Hence, this function
         * must be overridden if such behavior is not of interest.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#id-ebv">XML Path Language (XPath) 2.0,
         * 2.4.3 Effective Boolean Value</a>
         */
        virtual bool evaluateEBV(const DynamicContext::Ptr &context) const;

        /**
         * Evaluates this Expression by sending its output to DynamicContext::outputReceiver().
         */
        virtual void evaluateToSequenceReceiver(const DynamicContext::Ptr &context) const;

        /**
         * @returns the expression's child expressions. For example, a function's
         * arguments is returned here.
         *
         * If this Expression has no operands, an empty list should be returned.
         */
        virtual Expression::List operands() const = 0;

        virtual void setOperands(const Expression::List &operands) = 0;

        /**
         * @returns the static type of this Expression. For example, an 'and' expression
         * have as static type xs:boolean
         */
        virtual SequenceType::Ptr staticType() const = 0;

        /**
         * Returns a list of Sequence Types, describing the type of each of the
         * expression's operands. Hence, this function has a relationship to
         * the operands() function:
         *
         *     - The lengths of the lists returned by expectedOperandTypes()
         *     and operands() should always be equal in length, since one
         *     cannot describe the type of a non-existent operand(and all
         *     operands must have type information).
         *     - A significant difference between the two functions is that while
         *     the type of objects in the list returned by operands() may vary
         *     between compilations/static context, simply because the particular
         *     Expression is part of different XPath expressions, the
         *     types in the list returned by expectedOperandTypes is always the same
         *     since the function/operator signature never changes.
         *
         * This function should not be confused with staticType(),
         * which returns the static type of the expression itself, not its operands. The
         * function call is an expression where this is clear: the type of the return
         * value is not the same as the arguments' types. The static type of the
         * operands supplied to the expression can be determined via the staticType()
         * function of the instances returned by operands().
         *
         * If the expression has no operands, an empty list should be returned.
         */
        virtual SequenceType::List expectedOperandTypes() const = 0;

        /**
         * This implementation guarantees to never rewrite away this Expression, but
         * at most rewrite it as a child of another expression(that presumably have a
         * type checking role). It is therefore always safe to override this
         * function and call this implementation and not worry about that this Expression
         * becomes deleted.
         *
         * Many Expressions override typeCheck() and performs optimizations, as opposed
         * to doing it in the compress() stage. This is due to that the design
         * of those Expressions often are tied to that certain simplifications
         * are done at the typeCheck() stage of the compilation process or that
         * it in some other way is related to what the typeCheck() do. Also, the earlier
         * the AST can be simplified, the better the chances are for subsequent
         * optimizations.
         *
         * It is important that the super class's typeCheck() is called before doing
         * any custom type checking, since the call can change the children(notably,
         * the childrens' static types). For example, if the Expression, MyExpression
         * in the example, does not match the required type, typeCheck returns the Expression
         * wrapped in for example ItemVerifier, CardinalityVerifier, or both.
         *
         * typeCheck() may be called many times. typeCheck() must either raise an error
         * if this Expression is an invalid expression. Thus, it is guaranteed that an Expression
         * is valid after typeCheck() is called.
         *
         * @param context supplies information, such as namespace bindings and
         * available function signatures, that can be needed at compilation time. @p context is
         * guaranteed by the caller to never null.
         * @param reqType the static type that this Expression must match when evaluated. @p reqType is
         * guaranteed by the caller to never null.
         * @returns an Expression that can be this Expression, or another expression,
         * which somehow is necessary for making this Expression conforming to
         * @p reqType
         */
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        /**
         * compress() is the last stage performs in compiling an expression, done after
         * the initial AST build and calling typeCheck(). compress() performs crucial
         * simplifications, either by having drastic performance implications or that
         * some expressions depend on it for proper behavior.
         *
         * The default implementation performs a sparse conditional constant
         * propagation. In short, a recursive process is performed in the AST
         * which examines if the Expression's operands are constant values, and if so,
         * performs a const fold(AST rewrite) into the result of evaluating the expression
         * in question. This default behavior can be disabled by letting properties() return
         * DisableElimination.
         *
         * This compress() stage can be relative effective due to the design of XPath, in
         * part because intrinsic functions are heavily used. Many Expressions override compress()
         * and do optimizations specific to what they do. Also, many Expressions performs
         * optimizations in their typeCheck().
         *
         * @param context the static context. Supplies compile time information, and is
         * the channel for communicating error messages.
         * @see <a href="http://en.wikipedia.org/wiki/Sparse_conditional_constant_propagation">Wikipedia,
         * the free encyclopedia, Sparse conditional constant propagation</a>
         * @see <a href="http://en.wikipedia.org/wiki/Intrinsic_function">Wikipedia,
         * the free encyclopedia, Intrinsic function</a>
         * @see <a href="http://en.wikipedia.org/wiki/Compiler_optimization">Wikipedia, the
         * free encyclopedia, Compiler optimization</a>
         */
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

        /**
         * @returns a bitwise OR'd value of properties, describing the
         * characteristics of the expression. These properties affects how
         * this Expression is treated in for example type checking stages.
         *
         * The default implementation returns 0. Override and let the function return
         * a different value, if that's of interest.
         *
         * An important decision when re-implementing properties() is whether
         * to OR in the properties() of ones operands. For instance, if an
         * operand has RequiresFocus set, that flag nost likely applies to the
         * apparent as well, since it depends on its operand.
         *
         * @see deepProperties()
         * @returns Expression::None, meaning no special properties
         */
        virtual Properties properties() const;

        /**
         * Recursively computes through all descendants until a Property
         * is encount
         */
        virtual Properties dependencies() const;

        /**
         * @short Computes the union of properties for this Expression and all
         * its descending children.
         *
         * @see properties()
         */
        Properties deepProperties() const;

        /**
         * This function is a utility function, which performs bitwise logic
         * on properties() in order to find out whether the Expression::IsEvaluated
         * flag is set.
         *
         * @note Do not attempt to re-implement this function. Instead, return the
         * IsEvaluated flag by re-implementing the properties() function.
         */
        inline bool isEvaluated() const;

        /**
         * This function is a utility function, syntactic sugar for determining
         * whether this Expression is @p id. For example, calling <tt>is(IDIfThenClause)</tt>
         * is equivalent to <tt>id() == IDIfThenClause</tt>
         *
         * @note Do not attempt to re-implement this function. Instead, return the
         * appropriate flag in the virtual id() function.
         */
        inline bool is(const ID id) const;

        /**
         * Determines whether this Expression has Property @p prop set.
         *
         * Calling <tt>expr->has(MyProperty)</tt> is semantically equivalent
         * to <tt>expr->properties().testFlag(MyProperty)</tt>. In
         * other words, has(), as well as is(), provides syntacti sugar
         * and makes code more readable.
         *
         * @note Do not attempt to re-implement this function. Instead, return
         * the appropriate flag by re-implementing the properties() function.
         */
        inline bool has(const Property prop) const;

        inline bool hasDependency(const Property prop) const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const = 0;

        /**
         * This property, which has no setter, returns an enum value that uniquely identifies
         * this Expression. Patternist makes no use of C++'s dynamic_cast feature, but uses this
         * polymorphic function instead.
         *
         * @returns always IgnorableExpression.
         */
        virtual ID id() const;

        /**
         * Returns the OptimizationPasses that applies for this Expression. The
         * default implementation returns an empty list. Sub-classes can re-implement
         * this function and return actual OptimizationPasses.
         *
         * @returns always an empty list.
         */
        virtual QList<QExplicitlySharedDataPointer<OptimizationPass> > optimizationPasses() const;

        /**
         * Returns the required type the context item must be an instance of.
         *
         * If this Expression requires a focus, meaning its properties()
         * function returns RequiresContextItem,
         * it must return a type from this function. If any type is ok, BuiltinTypes::item should be
         * returned.
         *
         * In other words, this function must only be re-implemented if the focus is used. The default
         * implementation performs an assert crash.
         */
        virtual ItemType::Ptr expectedContextItemType() const;

        /**
         * If an Expression creates a focus because it has set the property CreatesFocusForLast,
         * it should override this function and make it return the ItemType that
         * the context item in the focus has.
         *
         * @returns never @c null.
         * @see announceFocusType()
         */
        virtual ItemType::Ptr newFocusType() const;

        /**
         * @short Returns @c this.
         */
        virtual const SourceLocationReflection *actualReflection() const;

        /**
         * Reimplementation of SourceLocationReflection::description().
         */
        virtual QString description() const;

        /**
         * When this function is called, it signals that the parent will create
         * a focus of type @p itemType.
         *
         * This type can also be retrieved through StaticContext::contextItemType()
         * when inside typeCheck(), but in some cases this is too late. For
         * instance, a parent needs to have the static type of its child
         * properly reported before it calls its typeCheck()(and the child's
         * type is inferred from the focus).
         *
         * The default implementation delegates the call on to the children.
         *
         * This function may be called at arbitrary times, in arbitrary
         * amounts.
         *
         * If the AST node overriding this call has children, it should be
         * considered whether the default implementation should be called, such
         * that they type is announced to them too.
         *
         * The caller guarantees that @p itemType is not @c null.
         */
        virtual void announceFocusType(const ItemType::Ptr &itemType);

        /**
         * This function take the two Expression pointers @p old and @p New, and
         * in a safe way, by handling reference counting and being aware of whether
         * the two pointers actually are different, switches the two. When compiling
         * in debug mode, informative debug messages are printed.
         *
         * This function is conceptually similar to Qt's qSwap(), but has
         * debugging functionality and also handles source locations.
         */
        static inline void rewrite(Expression::Ptr &old,
                                   const Expression::Ptr &New,
                                   const StaticContext::Ptr &context);

        /**
         * @short Rewrites this Expression to @p to, and return @p to.
         *
         * Source location annotations are adjusted appropriately.
         */
        inline const Expression::Ptr &rewrite(const Expression::Ptr &to,
                                              const StaticContext::Ptr &context) const;

        /**
         * By default 0.5 is returned.
         */
        virtual PatternPriority patternPriority() const;

    protected:

        /**
         * @returns @c true if all operands are constant values of somekind, and are already
         * evaluated. A string literal, is a typical example.
         */
        virtual bool compressOperands(const StaticContext::Ptr &) = 0;

        void typeCheckOperands(const StaticContext::Ptr &context);

    private:
        static Expression::Ptr invokeOptimizers(const Expression::Ptr &expr,
                                                const StaticContext::Ptr &context);
        /**
         * @return a StaticContext that has adopted the context item type properly
         * for this Expression.
         */
        inline StaticContext::Ptr finalizeStaticContext(const StaticContext::Ptr &context) const;

        /**
         * @short Performs constant propagation, also called constant folding, on this expression.
         *
         * This means that it attempts to evaluate this expression at compile and returns the result value
         * appropriately as an Expression. For example, for the XPath expression
         * <tt>1 + 3</tt> would an Integer of value 4 would be returned.
         *
         * It is not checked whether constant propagation is possible, the
         * caller is responsible for this.
         *
         * @see <a href="http://en.wikipedia.org/wiki/Constant_propagation">Constant folding,
         * From Wikipedia, the free encyclopedia</a>
         */
        Expression::Ptr constantPropagate(const StaticContext::Ptr &context) const;

        Q_DISABLE_COPY(Expression)
    };

    Q_DECLARE_OPERATORS_FOR_FLAGS(Expression::Properties)

    inline bool Expression::is(const Expression::ID i) const
    {
        return id() == i;
    }

    inline bool Expression::isEvaluated() const
    {
        return has(IsEvaluated);
    }

    inline bool Expression::has(const Expression::Property prop) const
    {
        return properties().testFlag(prop);
    }

    inline bool Expression::hasDependency(const Expression::Property prop) const
    {
        return dependencies().testFlag(prop);
    }

    inline void Expression::rewrite(Expression::Ptr &old,
                                    const Expression::Ptr &New,
                                    const StaticContext::Ptr &context)
    {
        Q_ASSERT(old);
        Q_ASSERT(New);

        if(old != New)
        {
            pDebug() << "AST REWRITE:" << old.data() << "to" << New.data()
                     << '(' << old->actualReflection() << "to" << New->actualReflection() << ", "
                     << old->description() << "to" << New->description() << ')';

            /* The order of these two lines is significant.. */
            context->addLocation(New.data(), context->locationFor(old->actualReflection()));
            old = New;
        }
    }

    inline const Expression::Ptr &Expression::rewrite(const Expression::Ptr &to,
                                                      const StaticContext::Ptr &context) const
    {
        context->addLocation(to.data(), context->locationFor(this));
        return to;
    }
}

Q_DECLARE_TYPEINFO(QPatternist::Expression::Ptr, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
