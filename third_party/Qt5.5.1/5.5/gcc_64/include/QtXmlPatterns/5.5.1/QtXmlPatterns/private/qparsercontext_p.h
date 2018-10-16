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

#ifndef Patternist_ParserContext_H
#define Patternist_ParserContext_H

#include <QFlags>
#include <QSharedData>
#include <QStack>
#include <QStringList>
#include <QtGlobal>
#include <QXmlQuery>

#include <private/qbuiltintypes_p.h>
#include <private/qfunctionsignature_p.h>
#include <private/qorderby_p.h>
#include <private/qtemplatemode_p.h>
#include <private/quserfunctioncallsite_p.h>
#include <private/quserfunction_p.h>
#include <private/qvariabledeclaration_p.h>
#include <private/qtokenvalue_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class Tokenizer;

    /**
     * @short Contains data used when parsing and tokenizing.
     *
     * When ExpressionFactory::create() is called, an instance of this class
     * is passed to the scanner and parser. It holds all information that is
     * needed to create the expression.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ParserContext : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ParserContext> Ptr;

        enum PrologDeclaration
        {
            BoundarySpaceDecl               = 1,
            DefaultCollationDecl            = 2,
            BaseURIDecl                     = 4,
            ConstructionDecl                = 8,
            OrderingModeDecl                = 16,
            EmptyOrderDecl                  = 32,
            CopyNamespacesDecl              = 64,
            DeclareDefaultElementNamespace  = 128,
            DeclareDefaultFunctionNamespace = 256
        };

        typedef QFlags<PrologDeclaration> PrologDeclarations;

        /**
         * Constructs a ParserContext instance.
         *
         * @param context the static context as defined in XPath. This contain
         * namespace bindings, error handler, and other information necessary
         * for creating an XPath expression.
         * @param lang the particular XPath language sub-set that should be parsed
         * @param tokenizer the Tokenizer to use.
         * @see ExpressionFactory::LanguageAccent
         */
        ParserContext(const StaticContext::Ptr &context,
                      const QXmlQuery::QueryLanguage lang,
                      Tokenizer *const tokenizer);

        /**
         * @short Removes the recently pushed variables from
         * scope. The amount of removed variables is @p amount.
         *
         * finalizePushedVariable() can be seen as popping the variable.
         *
         */
        void finalizePushedVariable(const int amount = 1,
                                    const bool shouldPop = true);

        inline VariableSlotID allocatePositionalSlot()
        {
            ++m_positionSlot;
            return m_positionSlot;
        }

        inline VariableSlotID allocateExpressionSlot()
        {
            const VariableSlotID retval = m_expressionSlot;
            ++m_expressionSlot;
            return retval;
        }

        inline VariableSlotID allocateGlobalVariableSlot()
        {
            ++m_globalVariableSlot;
            return m_globalVariableSlot;
        }

        inline bool hasDeclaration(const PrologDeclaration decl) const
        {
            return m_prologDeclarations.testFlag(decl);
        }

        inline void registerDeclaration(const PrologDeclaration decl)
        {
            m_prologDeclarations |= decl;
        }

        /**
         * The namespaces declared with <tt>declare namespace</tt>.
         */
        QStringList declaredPrefixes;

        /**
         * This is a temporary stack, used for keeping variables in scope,
         * such as for function arguments & let clauses.
         */
        VariableDeclaration::Stack variables;

        inline bool isXSLT() const
        {
            return languageAccent == QXmlQuery::XSLT20;
        }

        const StaticContext::Ptr staticContext;
        /**
         * We don't store a Tokenizer::Ptr here, because then we would get a
         * circular referencing between ParserContext and XSLTTokenizer, and
         * hence they would never destruct.
         */
        Tokenizer *const tokenizer;
        const QXmlQuery::QueryLanguage languageAccent;

        /**
         * Only used in the case of XSL-T. Is the name of the initial template
         * to call. If null, no name was provided, and regular template
         * matching should be done.
         */
        QXmlName initialTemplateName;

        /**
         * Used when parsing direct element constructors. It is used
         * for ensuring tags are well-balanced.
         */
        QStack<QXmlName> tagStack;

        /**
         * The actual expression, the Query. This member may be @c null,
         * such as in the case of an XQuery library module.
         */
        Expression::Ptr queryBody;

        /**
         * The user functions declared in the prolog.
         */
        UserFunction::List userFunctions;

        /**
         * Contains all calls to user defined functions.
         */
        UserFunctionCallsite::List userFunctionCallsites;

        /**
         * All variables declared with <tt>declare variable</tt>.
         */
        VariableDeclaration::List declaredVariables;

        QVector<qint16> parserStack_yyss;
        QVector<TokenValue> parserStack_yyvs;
        QVector<YYLTYPE> parserStack_yyls;

        void handleStackOverflow(const char*, short **yyss, size_t, TokenValue **yyvs, size_t, YYLTYPE **yyls, size_t, size_t *yystacksize);

        inline VariableSlotID currentPositionSlot() const
        {
            return m_positionSlot;
        }

        inline VariableSlotID currentExpressionSlot() const
        {
            return m_expressionSlot;
        }

        inline void restoreNodeTestSource()
        {
            nodeTestSource = BuiltinTypes::element;
        }

        inline VariableSlotID allocateCacheSlot()
        {
            return ++m_evaluationCacheSlot;
        }

        inline VariableSlotID allocateCacheSlots(const int count)
        {
            const VariableSlotID retval = m_evaluationCacheSlot + 1;
            m_evaluationCacheSlot += count + 1;
            return retval;
        }

        ItemType::Ptr nodeTestSource;

        QStack<Expression::Ptr> typeswitchSource;

        /**
         * The library module namespace set with <tt>declare module</tt>.
         */
        QXmlName::NamespaceCode moduleNamespace;

        /**
         * When a direct element constructor is processed, resolvers are
         * created in order to carry the namespace declarations. In such case,
         * the old resolver is pushed here.
         */
        QStack<NamespaceResolver::Ptr> resolvers;

        /**
         * This is used for handling the following obscene case:
         *
         * - <tt>\<e\>{1}{1}\<\/e\></tt> produce <tt>\<e\>11\</e\></tt>
         * - <tt>\<e\>{1, 1}\<\/e\></tt> produce <tt>\<e\>1 1\</e\></tt>
         *
         * This boolean tracks whether the previous reduction inside element
         * content was done with an enclosed expression.
         */
        bool isPreviousEnclosedExpr;

        int elementConstructorDepth;

        QStack<bool> scanOnlyStack;

        QStack<OrderBy::Stability> orderStability;

        /**
         * Whether any prolog declaration that must occur after the first
         * group has been encountered.
         */
        bool hasSecondPrologPart;

        bool preserveNamespacesMode;
        bool inheritNamespacesMode;

        /**
         * Contains all named templates. Since named templates
         * can also have rules, each body may also be in templateRules.
         */
        QHash<QXmlName, Template::Ptr>  namedTemplates;

        /**
         * All the @c xsl:call-template instructions that we have encountered.
         */
        QVector<Expression::Ptr>         templateCalls;

        /**
         * If we're in XSL-T, and a variable reference is encountered
         * which isn't in-scope, it's added to this hash since a global
         * variable declaration may appear later on.
         *
         * We use a multi hash, since we can encounter several references to
         * the same variable before it's declared.
         */
        QMultiHash<QXmlName, Expression::Ptr> unresolvedVariableReferences;

        /**
         *
         * Contains the encountered template rules, as opposed
         * to named templates.
         *
         * The key is the name of the template mode. If it's a default
         * constructed value, it's the default mode.
         *
         * Since templates rules may also be named, each body may also be in
         * namedTemplates.
         *
         * To be specific, the values are not the templates, the values are
         * modes, and the TemplateMode contains the patterns and bodies.
         */
        QHash<QXmlName, TemplateMode::Ptr>  templateRules;

        /**
         * @short Returns the TemplateMode for @p modeName or @c null if the
         * mode being asked for is @c #current.
         */
        TemplateMode::Ptr modeFor(const QXmlName &modeName)
        {
            /* #current is not a mode, so it cannot contain templates. #current
             * specifies how to look up templates wrt. mode. This check helps
             * code that calls us, asking for the mode it needs to lookup in.
             */
            if(modeName == QXmlName(StandardNamespaces::InternalXSLT, StandardLocalNames::current))
                return TemplateMode::Ptr();

            TemplateMode::Ptr &mode = templateRules[modeName];

            if(!mode)
                mode = TemplateMode::Ptr(new TemplateMode(modeName));

            Q_ASSERT(templateRules[modeName]);
            return mode;
        }

        inline TemplatePattern::ID allocateTemplateID()
        {
            ++m_currentTemplateID;
            return m_currentTemplateID;
        }

        /**
         * The @c xsl:param appearing inside template.
         */
        VariableDeclaration::List templateParameters;

        /**
         * The @c xsl:with-param appearing in template calling instruction.
         */
        WithParam::Hash templateWithParams;

        inline void templateParametersHandled()
        {
            finalizePushedVariable(templateParameters.count());
            templateParameters.clear();
        }

        inline void templateWithParametersHandled()
        {
            templateWithParams.clear();
        }

        inline bool isParsingWithParam() const
        {
            return m_isParsingWithParam.top();
        }

        void startParsingWithParam()
        {
            m_isParsingWithParam.push(true);
        }

        void endParsingWithParam()
        {
            m_isParsingWithParam.pop();
        }

        /**
         * This is used to deal with XSL-T's exception to the @c node() type,
         * which doesn't match document nodes.
         */
        bool                                isParsingPattern;

        ImportPrecedence                    currentImportPrecedence;

        bool isFirstTemplate() const
        {
            return m_currentTemplateID == InitialTemplateID;
        }

        /**
         * Whether we're processing XSL-T 1.0 code.
         */
        QStack<bool> isBackwardsCompat;

    private:
        enum
        {
            InitialTemplateID = -1
        };

        VariableSlotID                      m_evaluationCacheSlot;
        VariableSlotID                      m_expressionSlot;
        VariableSlotID                      m_positionSlot;
        PrologDeclarations                  m_prologDeclarations;
        VariableSlotID                      m_globalVariableSlot;
        TemplatePattern::ID                 m_currentTemplateID;

        /**
         * The default is @c false. If we're not parsing @c xsl:with-param,
         * hence parsing @c xsl:param, the value has changed.
         */
        QStack<bool>                        m_isParsingWithParam;
        Q_DISABLE_COPY(ParserContext)
    };
}

QT_END_NAMESPACE

#endif
