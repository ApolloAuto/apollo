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

#ifndef Patternist_XsdSchema_H
#define Patternist_XsdSchema_H

#include <private/qschematype_p.h>
#include <private/qxsdannotated_p.h>
#include <private/qxsdattribute_p.h>
#include <private/qxsdattributegroup_p.h>
#include <private/qxsdcomplextype_p.h>
#include <private/qxsdelement_p.h>
#include <private/qxsdidentityconstraint_p.h>
#include <private/qxsdmodelgroup_p.h>
#include <private/qxsdnotation_p.h>
#include <private/qxsdsimpletype_p.h>

#include <QtCore/QHash>
#include <QtCore/QReadWriteLock>

/**
 * @defgroup Patternist_schema XML Schema Processing
 */

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a XSD schema object.
     *
     * The class provides access to all components of a parsed XSD.
     *
     * @note In the documentation of this class objects, which are direct
     *       children of the <em>schema</em> object, are called top-level objects.
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSModel">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchema : public QSharedData, public XsdAnnotated
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdSchema> Ptr;
            typedef QList<XsdSchema::Ptr> List;

            /**
             * Creates a new schema object.
             *
             * @param namePool The namepool that should be used for names of
             *                 all schema components.
             */
            XsdSchema(const NamePool::Ptr &namePool);

            /**
             * Destroys the schema object.
             */
            ~XsdSchema();

            /**
             * Returns the namepool that is used for names of
             * all schema components.
             */
            NamePool::Ptr namePool() const;

            /**
             * Sets the @p targetNamespace of the schema.
             */
            void setTargetNamespace(const QString &targetNamespace);

            /**
             * Returns the target namespace of the schema.
             */
            QString targetNamespace() const;

            /**
             * Adds a new top-level @p element to the schema.
             *
             * @param element The new element.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-element">Element Declaration</a>
             */
            void addElement(const XsdElement::Ptr &element);

            /**
             * Returns the top-level element of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            XsdElement::Ptr element(const QXmlName &name) const;

            /**
             * Returns the list of all top-level elements.
             */
            XsdElement::List elements() const;

            /**
             * Adds a new top-level @p attribute to the schema.
             *
             * @param attribute The new attribute.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-attribute">Attribute Declaration</a>
             */
            void addAttribute(const XsdAttribute::Ptr &attribute);

            /**
             * Returns the top-level attribute of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            XsdAttribute::Ptr attribute(const QXmlName &name) const;

            /**
             * Returns the list of all top-level attributes.
             */
            XsdAttribute::List attributes() const;

            /**
             * Adds a new top-level @p type to the schema.
             * That can be a simple or a complex type.
             *
             * @param type The new type.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-datatype">Simple Type Declaration</a>
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-type">Complex Type Declaration</a>
             */
            void addType(const SchemaType::Ptr &type);

            /**
             * Returns the top-level type of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            SchemaType::Ptr type(const QXmlName &name) const;

            /**
             * Returns the list of all top-level types.
             */
            SchemaType::List types() const;

            /**
             * Returns the list of all top-level simple types.
             */
            XsdSimpleType::List simpleTypes() const;

            /**
             * Returns the list of all top-level complex types.
             */
            XsdComplexType::List complexTypes() const;

            /**
             * Adds an anonymous @p type to the schema.
             * Anonymous types have no name and are declared
             * locally inside an element object.
             *
             * @param type The new anonymous type.
             */
            void addAnonymousType(const SchemaType::Ptr &type);

            /**
             * Returns the list of all anonymous types.
             */
            SchemaType::List anonymousTypes() const;

            /**
             * Adds a new top-level attribute @p group to the schema.
             *
             * @param group The new attribute group.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-attributeGroup">Attribute Group Declaration</a>
             */
            void addAttributeGroup(const XsdAttributeGroup::Ptr &group);

            /**
             * Returns the top-level attribute group of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            XsdAttributeGroup::Ptr attributeGroup(const QXmlName name) const;

            /**
             * Returns the list of all top-level attribute groups.
             */
            XsdAttributeGroup::List attributeGroups() const;

            /**
             * Adds a new top-level element @p group to the schema.
             *
             * @param group The new element group.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-namedModelGroup">Element Group Declaration</a>
             */
            void addElementGroup(const XsdModelGroup::Ptr &group);

            /**
             * Returns the top-level element group of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            XsdModelGroup::Ptr elementGroup(const QXmlName &name) const;

            /**
             * Returns the list of all top-level element groups.
             */
            XsdModelGroup::List elementGroups() const;

            /**
             * Adds a new top-level @p notation to the schema.
             *
             * @param notation The new notation.
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#declare-notation">Notation Declaration</a>
             */
            void addNotation(const XsdNotation::Ptr &notation);

            /**
             * Returns the top-level notation of the schema with
             * the given @p name or an empty pointer if none exist.
             */
            XsdNotation::Ptr notation(const QXmlName &name) const;

            /**
             * Returns the list of all top-level notations.
             */
            XsdNotation::List notations() const;

            /**
             * Adds a new identity @p constraint to the schema.
             */
            void addIdentityConstraint(const XsdIdentityConstraint::Ptr &constraint);

            /**
             * Returns the identity constraint with the given @p name
             * or an empty pointer if none exist.
             */
            XsdIdentityConstraint::Ptr identityConstraint(const QXmlName &name) const;

            /**
             * Returns the list of all identity constraints in this schema.
             */
            XsdIdentityConstraint::List identityConstraints() const;

        private:
            NamePool::Ptr                               m_namePool;
            QString                                     m_targetNamespace;
            QHash<QXmlName, XsdElement::Ptr>            m_elements;
            QHash<QXmlName, XsdAttribute::Ptr>          m_attributes;
            QHash<QXmlName, SchemaType::Ptr>            m_types;
            QHash<QXmlName, SchemaType::Ptr>            m_anonymousTypes;
            QHash<QXmlName, XsdAttributeGroup::Ptr>     m_attributeGroups;
            QHash<QXmlName, XsdModelGroup::Ptr>         m_elementGroups;
            QHash<QXmlName, XsdNotation::Ptr>           m_notations;
            QHash<QXmlName, XsdIdentityConstraint::Ptr> m_identityConstraints;
            mutable QReadWriteLock                      m_lock;
    };
}

QT_END_NAMESPACE

#endif
