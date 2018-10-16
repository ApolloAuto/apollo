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

#ifndef Patternist_StringValueFNs_H
#define Patternist_StringValueFNs_H

#include <QByteArray>

#include <private/qfunctioncall_p.h>

/**
 * @file
 * @short Contains classes implementing the functions found in
 * <a href="http://www.w3.org/TR/xpath-functions/#string-value-functions">XQuery 1.0 and
 * XPath 2.0 Functions and Operators, 7.4 Functions on AtomicString Values</a>.
 *
 * @ingroup Patternist_functions
 */

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Implements the function <tt>fn:concat()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ConcatFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:string-join()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class StringJoinFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;

        /**
         * Optimization: when the cardinality of the sequence of items to join
         * cannot be two or more, we have no effect and therefore rewrite
         * ourselves to our first operand.
         */
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);
    };

    /**
     * @short Implements the function <tt>fn:substring()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class SubstringFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:string-length()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class StringLengthFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:normalize-space()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class NormalizeSpaceFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:normalize-unicode()</tt>.
     *
     * What perhaps can be said significant with the implementation, is that it
     * attempts to determine the normalization form at compile time, in order to
     * reduce string work at runtime.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class NormalizeUnicodeFN : public FunctionCall
    {
    public:
        /**
         * Initializes private data.
         */
        NormalizeUnicodeFN();
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

    private:
        int determineNormalizationForm(const DynamicContext::Ptr &context) const;
        QString::NormalizationForm m_normForm;
    };

    /**
     * @short Implements the function <tt>fn:upper-case()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class UpperCaseFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:lower-case()</tt>.
     *
     * @short Implements the function <tt>fn:concat()</tt>.
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class LowerCaseFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Implements the function <tt>fn:translate()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class TranslateFN : public FunctionCall
    {
    public:
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
    };

    /**
     * @short Provides functionality for encoding strings. Sub-classed by various
     * function implementations.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class EncodeString : public FunctionCall
    {
    public:
        /**
         * Evaluates its first operand. If it is the empty sequence, an empty string
         * is returned. Otherwise, the item's string value is returned percent encoded
         * as specified in this class's constructor.
         */
        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;

    protected:
        /**
         * Encodes its operand with QUrl::toPercentEncoding(), with @p includeChars as
         * the characters to encode, and @p excludeChars as the characters to not encode.
         */
        EncodeString(const QByteArray &excludeChars, const QByteArray &includeChars);

    private:
        const QByteArray m_excludeChars;
        const QByteArray m_includeChars;
    };

    /**
     * @short Implements the function <tt>fn:encode-for-uri()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class EncodeForURIFN : public EncodeString
    {
    public:
        /**
         * Performs internal initialization.
         */
        EncodeForURIFN();

    private:
        static const char *const include;
    };

    /**
     * @short Implements the function <tt>fn:iri-to-uri()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class IriToURIFN : public EncodeString
    {
    public:
        /**
         * Performs internal initialization.
         */
        IriToURIFN();

    private:
        static const char *const exclude;
    };

    /**
     * @short Implements the function <tt>fn:escape-html-uri()</tt>.
     *
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class EscapeHtmlURIFN : public EncodeString
    {
    public:
        /**
         * Performs internal initialization.
         */
        EscapeHtmlURIFN();

    private:
        static const char *const include;
        static const char *const exclude;
    };
}

QT_END_NAMESPACE

#endif
