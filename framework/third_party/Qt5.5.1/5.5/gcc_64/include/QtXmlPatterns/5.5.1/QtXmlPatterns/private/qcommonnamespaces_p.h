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

#ifndef Patternist_CommonNamespaces_H
#define Patternist_CommonNamespaces_H

#include <QLatin1String>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Contains common, standardized XML namespaces.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    namespace CommonNamespaces
    {
        /**
         * Namespace for the special XML namespace. It is by definition
         * bound to the "xml" prefix, and should have no usage in
         * ordinary code.
         *
         * Specification: http://www.w3.org/TR/REC-xml-names/
         */
        const QLatin1String XML("http://www.w3.org/XML/1998/namespace");

        /**
         * The namespace for the xmlns prefix. The Namespaces in XML recommendation
         * explicitly states that xmlns should not have a namespace, but has since
         * been changed. See:
         *
         * http://www.w3.org/2000/xmlns/
         */
        const QLatin1String XMLNS("http://www.w3.org/2000/xmlns/");

        /**
         * The namespace for W3C XML Schema. This is used for the XML language it
         * is, as well as its built-in data types.
         *
         * Specification: http://www.w3.org/TR/xmlschema-2/
         * @see <a href="http://www.w3.org/TR/xmlschema-2/datatypes.html#namespaces">XML Schema
         * Part 2: Datatypes Second Edition, 3.1 Namespace considerations</a>
         */
        const QLatin1String WXS("http://www.w3.org/2001/XMLSchema");

        /**
         * The namespace for W3C XML Schema attributes used in schema instances.
         *
         * Specification: http://www.w3.org/TR/xmlschema-2/
         *
         * @see <a href="http://www.w3.org/TR/xmlschema-1/structures.html#Instance_Document_Constructions">XML
         * Schema Part 1: Structures Second Edition, 2.6 Schema-Related
         * Markup in Documents Being Validated</a>
         */
        const QLatin1String XSI("http://www.w3.org/2001/XMLSchema-instance");

        /**
         * The namespace for built-in XPath functions, as defined in for example
         * XQuery 1.0 and XPath 2.0 Functions and Operators and XSLT.
         *
         * Specification: http://www.w3.org/TR/xquery-operators/
         */
        const QLatin1String XFN("http://www.w3.org/2005/xpath-functions");

        /**
         * The namespace for XSL-T 1.0 and 2.0.
         *
         * @see <a href="http://www.w3.org/TR/xslt20/#xslt-namespace">XSL
         * Transformations (XSLT) Version 2.0, 3.1 XSLT Namespace</a>
         * @see <a href="http://www.w3.org/TR/xslt">XSL Transformations (XSLT) Version 1.0</a>
         */
        const QLatin1String XSLT("http://www.w3.org/1999/XSL/Transform");

        /**
         * The namespace for identifying errors in XPath.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#id-identifying-errors">XML Path Language (XPath)
         * 2.0, 2.3.2 Identifying and Reporting Errors</a>
         */
        const QLatin1String XPERR("http://www.w3.org/2005/xqt-errors");

        /**
         * The XPath 2.0 Unicode codepoint collation URI identifier. Collations
         * specifies how strings are compared and ordered.
         */
        const char *const UNICODE_COLLATION = "http://www.w3.org/2005/xpath-functions/collation/codepoint";

        /**
         * A namespace provided in XQuery 1.0, to easily declare local
         * variables and functions.
         */
        const QLatin1String XDT_LOCAL("http://www.w3.org/2005/xquery-local-functions");
    }
}

QT_END_NAMESPACE

#endif
