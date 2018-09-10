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

#ifndef Patternist_XsdIdentityConstraint_H
#define Patternist_XsdIdentityConstraint_H

#include <private/qnamedschemacomponent_p.h>
#include <private/qxsdannotated_p.h>
#include <private/qxsdxpathexpression_p.h>

#include <QtCore/QStringList>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a XSD identity constraint object.
     *
     * This class represents the <em>identity constraint</em> object of a XML schema as described
     * <a href="http://www.w3.org/TR/xmlschema11-1/#cIdentity-constraint_Definitions">here</a>.
     *
     * It contains information from either a <em>key</em> object, a <em>keyref</em> object or an
     * <em>unique</em> object.
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSIdentityConstraint">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdIdentityConstraint : public NamedSchemaComponent, public XsdAnnotated
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdIdentityConstraint> Ptr;
            typedef QList<XsdIdentityConstraint::Ptr> List;

            /**
             * Describes the <a href="http://www.w3.org/TR/xmlschema11-1/#icd-identity-constraint_category">category</a> of the identity constraint.
             */
            enum Category
            {
                Key = 1,        ///< The constraint is a key constraint
                KeyReference,   ///< The constraint is a keyref constraint
                Unique          ///< The constraint is an unique constraint
            };

            /**
             * Sets the @p category of the identity constraint.
             *
             * @see Category
             */
            void setCategory(Category category);

            /**
             * Returns the category of the identity constraint.
             */
            Category category() const;

            /**
             * Sets the @p selector of the identity constraint.
             *
             * The selector is a restricted XPath 1.0 expression,
             * that selects a set of nodes.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#icd-selector"></a>
             */
            void setSelector(const XsdXPathExpression::Ptr &selector);

            /**
             * Returns the selector of the identity constraint.
             */
            XsdXPathExpression::Ptr selector() const;

            /**
             * Sets the @p fields of the identity constraint.
             *
             * Each field is a restricted XPath 1.0 expression,
             * that selects a set of nodes.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#icd-fields"></a>
             */
            void setFields(const XsdXPathExpression::List &fields);

            /**
             * Adds a new @p field to the identity constraint.
             */
            void addField(const XsdXPathExpression::Ptr &field);

            /**
             * Returns all fields of the identity constraint.
             */
            XsdXPathExpression::List fields() const;

            /**
             * Sets the referenced @p key of the identity constraint.
             *
             * The key points to a identity constraint of type Key or Unique.
             *
             * The identity constraint has only a referenced key if its
             * type is KeyReference.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#icd-referenced_key"></a>
             */
            void setReferencedKey(const XsdIdentityConstraint::Ptr &key);

            /**
             * Returns the referenced key of the identity constraint or an empty
             * pointer if its type is not KeyReference.
             */
            XsdIdentityConstraint::Ptr referencedKey() const;

        private:
            Category                   m_category;
            XsdXPathExpression::Ptr    m_selector;
            XsdXPathExpression::List   m_fields;
            XsdIdentityConstraint::Ptr m_referencedKey;
    };
}

QT_END_NAMESPACE

#endif
