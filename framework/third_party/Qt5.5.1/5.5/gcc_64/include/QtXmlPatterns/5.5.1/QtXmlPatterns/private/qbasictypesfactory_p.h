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

#ifndef Patternist_BuiltinTypesFactory_H
#define Patternist_BuiltinTypesFactory_H

#include <QHash>
#include <private/qschematypefactory_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Factory for creating schema types for the types defined in XSL-T 2.0.
     *
     * Theses types are essentially the builtin primitive types, plus @c xs:integer,
     * and the types defined in the XPath Data Model.
     *
     * @ingroup Patternist_types
     * @see <a href="http://www.w3.org/TR/xpath-datamodel/#types-predefined">XQuery 1.0 and
     * XPath 2.0 Data Model, 2.6.2 Predefined Types</a>
     * @see <a href="http://www.w3.org/TR/xslt20/#built-in-types">XSL Transformations (XSLT)
     * Version 2.0, 3.13 Built-in Types</a>
     * @see <a href="http://www.w3.org/TR/xmlschema-2/#built-in-primitive-datatypes">XML Schema
     * Part 2: Datatypes Second Edition, 3.2 Primitive datatypes</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class BasicTypesFactory : public SchemaTypeFactory
    {
    public:

        /**
         * Creates a primitive type for @p name. If @p name is not supported,
         * @c null is returned.
         * The intened supported types are the builtin primitive and derived types.
         * That is, the 19 W3C XML Schema types, and the additional 5 in the XPath Data MOdel.
         *
         * @note This does not handle user defined types, only builtin types.
         * @todo Update documentation, proportionally with progress.
         */
        virtual SchemaType::Ptr createSchemaType(const QXmlName ) const;

        virtual SchemaType::Hash types() const;

        /**
         * @returns the singleton instance of BasicTypesFactory.
         */
        static SchemaTypeFactory::Ptr self(const NamePool::Ptr &np);

    protected:
        /**
         * This constructor is protected. Use the static self() function
         * to retrieve a singleton instance.
         */
        BasicTypesFactory(const NamePool::Ptr &np);

    private:
        /**
         * A dictonary of builtin primitive and derived primitives.
         */
        SchemaType::Hash m_types;
    };
}

QT_END_NAMESPACE

#endif
