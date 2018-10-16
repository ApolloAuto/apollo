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

#ifndef Patternist_XSLTTokenizer_H
#define Patternist_XSLTTokenizer_H

#include <QQueue>
#include <QStack>
#include <QUrl>

#include <private/qmaintainingreader_p.h>
#include <private/qreportcontext_p.h>
#include <private/qtokenizer_p.h>
#include <private/qxslttokenlookup_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A TokenSource which contains one Tokenizer::Token.
     *
     * One possible way to optimize this is to let SingleTokenContainer
     * actually contain a list of tokens, such that XSLTTokenizer::queueToken()
     * could append to that, instead of instansiating a SingleTokenContainer
     * all the time.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class SingleTokenContainer : public TokenSource
    {
    public:
        inline SingleTokenContainer(const Tokenizer::Token &token,
                                    const YYLTYPE &location);

        virtual Tokenizer::Token nextToken(YYLTYPE *const sourceLocator);
    private:
        const Tokenizer::Token m_token;
        const YYLTYPE          m_location;
        bool                   m_hasDelivered;
    };

    SingleTokenContainer::SingleTokenContainer(const Tokenizer::Token &token,
                                               const YYLTYPE &location) : m_token(token)
                                                                        , m_location(location)
                                                                        , m_hasDelivered(false)
    {
    }

    /**
     * @short Tokenizes XSL-T 2.0 documents.
     *
     * XSLTTokenizer takes in its constructor a pointer to a QIODevice which is
     * supposed to contain an XSL-T document. XSLTTokenizer then rewrites that
     * document into XQuery tokens delivered via nextToken(), which the regular
     * XQuery parser then reads. Hence, the XSL-T language is rewritten into
     * XQuery code, slightly extended to handle the featuress specific to
     * XSL-T.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class XSLTTokenizer : public Tokenizer
                        , private MaintainingReader<XSLTTokenLookup>
    {
    public:
        /**
         * XSLTTokenizer do not own @p queryDevice.
         */
        XSLTTokenizer(QIODevice *const queryDevice,
                      const QUrl &location,
                      const ReportContext::Ptr &context,
                      const NamePool::Ptr &np);

        virtual Token nextToken(YYLTYPE *const sourceLocator);

        /**
         * For XSLT we don't need this mechanism, so we do nothing.
         */
        virtual int commenceScanOnly();

        /**
         * For XSLT we don't need this mechanism, so we do nothing.
         */
        virtual void resumeTokenizationFrom(const int position);

        virtual void setParserContext(const ParserContext::Ptr &parseInfo);

        virtual QUrl documentURI() const
        {
            return queryURI();
        }

    protected:
        virtual bool isAnyAttributeAllowed() const;

    private:
        inline void validateElement() const;

        YYLTYPE currentSourceLocator() const;

        enum State
        {
            OutsideDocumentElement,
            InsideStylesheetModule,
            InsideSequenceConstructor
        };

        enum VariableType
        {
            FunctionParameter,
            GlobalParameter,
            TemplateParameter,
            VariableDeclaration,
            VariableInstruction,
            WithParamVariable
        };

        void queueNamespaceDeclarations(TokenSource::Queue *const ts,
                                        QStack<Token> *const target,
                                        const bool isDeclaration = false);

        inline void queueToken(const Token &token,
                               TokenSource::Queue *const ts);
        void queueEmptySequence(TokenSource::Queue *const to);
        void queueSequenceType(const QString &expr);
        /**
         * If @p emptynessAllowed is @c true, the @c select attribute may
         * be empty while there also is no sequence constructor.
         */
        void queueSimpleContentConstructor(const ReportContext::ErrorCode code,
                                           const bool emptynessAllowed,
                                           TokenSource::Queue *const to,
                                           const bool selectOnlyFirst = false);
        /**
         * Tokenizes and queues @p expr as if it was an attribute value
         * template.
         */
        void queueAVT(const QString &expr,
                      TokenSource::Queue *const to);

        void hasWrittenExpression(bool &beacon);
        void commencingExpression(bool &hasWrittenExpression,
                                  TokenSource::Queue *const to);

        void outsideDocumentElement();
        void insideChoose(TokenSource::Queue *const to);
        void insideFunction();

        bool attributeYesNo(const QString &localName) const;

        /**
         * Scans/skips @c xsl:fallback elements only. This is the case of the
         * children of @c xsl:sequence, for instance.
         */
        void parseFallbacksOnly();

        /**
         * Returns true if the current element is either @c stylesheet
         * or the synonym @c transform.
         *
         * This function assumes that m_reader is positioned at an element
         * and that the namespace is XSL-T.
         */
        bool isStylesheetElement() const;

        /**
         * Returns true if the current element name is @p name.
         *
         * It is assumed that the namespace is XSL-T and that the current
         * state in m_reader is either QXmlStreamReader::StartElement or
         * QXmlStreamReader::EndElement.
         */
        bool isElement(const NodeName &name) const;

        /**
         * Queues a text constructor for @p chars, if @p chars is
         * not empty.
         */
        void queueTextConstructor(QString &chars,
                                  bool &hasWrittenExpression,
                                  TokenSource::Queue *const to);

        /**
         *
         * @see <a href="http://www.w3.org/TR/xslt20/#stylesheet-structure">XSL
         * Transformations (XSLT) Version 2, 3.6 Stylesheet Element</a>
         */
        void insideStylesheetModule();
        void insideTemplate();

        /**
         * Takes @p expr for an XPath expression, and pushes the necessary
         * things for having it delivered as a stream of token, appropriate
         * for Effective Boolean Value parsing.
         */
        void queueExpression(const QString &expr,
                             TokenSource::Queue *const to,
                             const bool wrapWithParantheses = true);

        void skipBodyOfParam(const ReportContext::ErrorCode code);

        void queueParams(const NodeName parentName,
                         TokenSource::Queue *const to);

        /**
         * Used for @c xsl:apply-templates and @c xsl:call-templates.
         */
        void queueWithParams(const NodeName parentName,
                             TokenSource::Queue *const to,
                             const bool initialAdvance = true);

        /**
         * Queues an @c xsl:variable declaration. If @p isInstruction is @c
         * true, it is assumed to be a an instruction, otherwise a top-level
         * declaration element.
         */
        void queueVariableDeclaration(const VariableType variableType,
                                      TokenSource::Queue *const to);

        /**
         * Skips the current sub-tree.
         *
         * If text nodes that aren't strippable whitespace, or elements are
         * encountered, @c true is returned, otherwise @c false.
         *
         * If @p exitOnContent is @c true, this function exits immediately
         * if content is encountered for which it would return @c false.
         */
        bool skipSubTree(const bool exitOnContent = false);

        /**
         * Queues the necessary tokens for the expression that is either
         * supplied using a @c select attribute or a sequence constructor,
         * while doing the necessary error handling for ensuring they are
         * mutually exclusive.
         *
         * It is assumed that the current state of m_reader is
         * QXmlStreamReader::StartElement, or that the attributes for the
         * element is supplied through @p atts. This function advances m_reader
         * up until the corresponding QXmlStreamReader::EndElement.
         *
         * If @p emptynessAllowed is @c false, the element must either have a
         * sequence constructor or a @c select attribute. If @c true, both may
         * be absent.
         *
         * Returns @c true if the queued expression was supplied through the
         * @c select attribute otherwise @c false.
         */
        bool queueSelectOrSequenceConstructor(const ReportContext::ErrorCode code,
                                              const bool emptynessAllowed,
                                              TokenSource::Queue *const to,
                                              const QXmlStreamAttributes *const atts = 0,
                                              const bool queueEmptyOnEmpty = true);

        /**
         * If @p initialAdvance is @c true, insideSequenceConstructor() will
         * advance m_reader, otherwise it won't. Not doing so is useful
         * when the caller is already inside a sequence constructor.
         *
         * Returns @c true if a sequence constructor was found and queued.
         * Returns @c false if none was found, and the empty sequence was
         * synthesized.
         */
        bool insideSequenceConstructor(TokenSource::Queue *const to,
                                       const bool initialAdvance = true,
                                       const bool queueEmptyOnEmpty = true);

        bool insideSequenceConstructor(TokenSource::Queue *const to,
                                       QStack<Token> &queueOnExit,
                                       const bool initialAdvance = true,
                                       const bool queueEmptyOnEmpty = true);

        void insideAttributeSet();
        void pushState(const State nextState);
        void leaveState();

        /**
         * @short Handles @c xml:space and standard attributes.
         *
         * If @p isXSLTElement is @c true, the current element is an XSL-T
         * element, as opposed to a Literal Result Element.
         *
         * handleStandardAttributes() must be called before validateElement(),
         * because the former determines the version in use, and
         * validateElement() depends on that.
         *
         * The core of this function can't be run many times because it pushes
         * whitespace handling onto m_stripWhitespace.
         * m_hasHandledStandardAttributes protects helping against this.
         *
         * @see validateElement()
         * @see <a href="http://www.w3.org/TR/xslt20/#standard-attributes">XSL
         * Transformations (XSLT) Version 2.0, 3.5 Standard Attributes</a>
         */
        void handleStandardAttributes(const bool isXSLTElement);

        /**
         * @short Sends the tokens in @p source to @p destination.
         */
        inline void queueOnExit(QStack<Token> &source,
                                TokenSource::Queue *const destination);

        /**
         * Handles the @c type and @c validation attribute on instructions and
         * literal result elements.
         *
         * @p isLRE should be true if the current element is not in the XSL-T
         * namespace, that is if it's a Literal Result Element.
         *
         * @see <a href="http://www.w3.org/TR/xslt20/#validation">XSL
         * Transformations (XSLT) Version 2.0, 19.2 Validation</a>
         */
        void handleValidationAttributes(const bool isLRE) const;

        void unexpectedContent(const ReportContext::ErrorCode code = ReportContext::XTSE0010) const;

        void checkForParseError() const;

        inline void startStorageOfCurrent(TokenSource::Queue *const to);
        inline void endStorageOfCurrent(TokenSource::Queue *const to);

        /**
         * Checks that @p attribute has a value in accordance with what
         * is allowed and supported.
         */
        void handleXSLTVersion(TokenSource::Queue *const to,
                               QStack<Token> *const queueOnExit,
                               const bool isXSLTElement,
                               const QXmlStreamAttributes *atts = 0,
                               const bool generateCode = true,
                               const bool setGlobalVersion = false);

        /**
         * @short Generates code for reflecting @c xml:base attributes.
         */
        void handleXMLBase(TokenSource::Queue *const to,
                           QStack<Token> *const queueOnExit,
                           const bool isInstruction = true,
                           const QXmlStreamAttributes *atts = 0);

        /**
         * Concatenates text nodes, ignores comments and processing
         * instructions, and raises errors on everything else.
         *
         * Hence, similar to QXmlStreamReader::readElementText(), except
         * for error handling.
         */
        QString readElementText();

        /**
         * Tokenizes and validate xsl:sort statements, if any, until
         * other content is encountered. The produced tokens are returned
         * in a list.
         *
         * If @p oneSortRequired, at least one @c sort element must appear,
         * otherwise an error is raised.
         *
         * If @p speciallyTreatWhitespace whitespace will be treated as if it
         * was one of the elements mentioned in step 4 in section 4.2 Stripping
         * Whitespace from the Stylesheet.
         */
        void queueSorting(const bool oneSortRequired,
                          TokenSource::Queue *const to,
                          const bool speciallyTreatWhitespace = false);

        static ElementDescription<XSLTTokenLookup>::Hash createElementDescriptions();
        static QHash<QString, int> createValidationAlternatives();
        static QSet<NodeName> createStandardAttributes();

        /**
         * Reads the attribute by name @p attributeName, and returns @c true if
         * its value is @p isTrue, @c false if it is @p isFalse, and raise an
         * error otherwise.
         */
        bool readToggleAttribute(const QString &attributeName,
                                 const QString &isTrue,
                                 const QString &isFalse,
                                 const QXmlStreamAttributes *const atts = 0) const;

        int readAlternativeAttribute(const QHash<QString, int> &alternatives,
                                     const QXmlStreamAttribute &attr) const;

        /**
         * Returns @c true if the current text node can be skipped without
         * it leading to a validation error, with respect to whitespace.
         */
        inline bool whitespaceToSkip() const;

        const QUrl                                  m_location;
        const NamePool::Ptr                         m_namePool;
        QStack<State>                               m_state;
        TokenSource::Queue                          m_tokenSource;

        enum ProcessMode
        {
            BackwardsCompatible,
            ForwardCompatible,
            NormalProcessing
        };

        /**
         * Whether we're processing in Forwards-Compatible or
         * Backwards-Compatible mode.
         *
         * This is set by handleStandardAttributes().
         *
         * ParserContext have similar information in
         * ParserContext::isBackwardsCompat. A big distinction is that both the
         * tokenizer and the parser buffer tokens and have positions disjoint
         * to each other. E.g, the state the parser has when reducing into
         * non-terminals, is different from the tokenizer's.
         */
        QStack<ProcessMode>                         m_processingMode;

        /**
         * Returns @c true if the current state in m_reader is in the XSLT
         * namespace. It is assumed that the current state is an element.
         */
        inline bool isXSLT() const;

        const QHash<QString, int>                   m_validationAlternatives;

        ParserContext::Ptr                          m_parseInfo;
    };
}

QT_END_NAMESPACE

#endif
