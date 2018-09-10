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

#ifndef Patternist_XPathHelper_H
#define Patternist_XPathHelper_H

#include <private/qcommonnamespaces_p.h>
#include <private/qitem_p.h>
#include <private/qpatternistlocale_p.h>
#include <private/qreportcontext_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Contains helper and utility functions.
     *
     * The common denominator of its functions is that they do not fit in
     * well elsewhere, such as in a particular class. It is preferred if XPathHelper
     * goes away, and that functions are in more specific classes.
     *
     * @ingroup Patternist
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class XPathHelper
    {
    public:
        /**
         * Determines whether @p qName is a valid QName. For example, "body" and "xhtml:body"
         * is, but "xhtml::body" or "x:body "(note the whitespace) is not.
         *
         * @see QNameConstructor::expandQName()
         * @see QNameValue
         */
        static bool isQName(const QString &qName);

        /**
         * @short Splits @p qName into @p localName and @p prefix.
         *
         * @note @p qName must be a valid QName, and that is not checked.
         */
        static void splitQName(const QString &qName, QString &prefix, QString &localName);

        /**
         * Determines whether @p ns is a reserved namespace.
         *
         * @see <a href="http://www.w3.org/TR/xslt20/#reserved-namespaces">XSL Transformations
         * (XSLT) Version 2.0, 3.2 Reserved Namespaces</a>
         * @see <a href="http://www.w3.org/TR/xquery/#FunctionDeclns">XQuery 1.0: An XML
         * Query Language, 4.15 Function Declaration</a>
         * @returns @c true if @p ns is a reserved namespace, otherwise @c false.
         */
        static bool isReservedNamespace(const QXmlName::NamespaceCode ns);

        /**
         * Determines whether @p collation is a supported string collation. If it is
         * not, error code @p code is raised via @p context.
         */
        template<const ReportContext::ErrorCode code, typename TReportContext>
        static inline void checkCollationSupport(const QString &collation,
                                                 const TReportContext &context,
                                                 const SourceLocationReflection *const r)
        {
            Q_ASSERT(context);
            Q_ASSERT(r);

            if(collation != QLatin1String(CommonNamespaces::UNICODE_COLLATION))
            {
                context->error(QtXmlPatterns::tr("Only the Unicode Codepoint "
                                  "Collation is supported(%1). %2 is unsupported.")
                                  .arg(formatURI(QLatin1String(CommonNamespaces::UNICODE_COLLATION)))
                                  .arg(formatURI(collation)),
                               code, r);
            }
        }

        static QPatternist::ItemTypePtr typeFromKind(const QXmlNodeModelIndex::NodeKind nodeKind);

        /**
         * Normalizes an @p uri by resolving it to the application directory if empty.
         */
        static QUrl normalizeQueryURI(const QUrl &uri);

        /**
         * @short Determines whether @p consists only of whitespace. Characters
         * considered whitespace are the ones for which QChar::isSpace() returns @c true for.
         *
         * For the empty string, @c true is returned.
         *
         * @returns @c true if @p string consists only of whitespace, otherwise @c false.
         */
        static inline bool isWhitespaceOnly(const QStringRef &string)
        {
            const int len = string.length();

            for(int i = 0; i < len; ++i)
            {
                if(!string.at(i).isSpace()) // TODO and this is wrong, see sdk/TODO
                    return false;
            }

            return true;
        }

        /**
         * @overload
         */
        static inline bool isWhitespaceOnly(const QString &string)
        {
            return isWhitespaceOnly(QStringRef(&string));
        }

    private:
        /**
         * @short This default constructor has no definition, in order to avoid
         * instantiation, since it makes no sense to instantiate this class.
         */
        inline XPathHelper();

        Q_DISABLE_COPY(XPathHelper)
    };
}

QT_END_NAMESPACE

#endif
