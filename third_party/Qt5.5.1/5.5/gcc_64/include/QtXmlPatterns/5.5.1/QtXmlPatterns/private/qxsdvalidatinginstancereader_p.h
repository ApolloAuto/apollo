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

#ifndef Patternist_XsdValidatingInstanceReader_H
#define Patternist_XsdValidatingInstanceReader_H

#include <private/qxsdidchelper_p.h>
#include <private/qxsdinstancereader_p.h>
#include <private/qxsdstatemachine_p.h>
#include <private/qxsdvalidatedxmlnodemodel_p.h>

#include <QtCore/QStack>

QT_BEGIN_NAMESPACE

class QXmlQuery;

namespace QPatternist
{
    /**
     * @short The validating schema instance reader.
     *
     * This class reads in a xml instance document from a QAbstractXmlNodeModel and
     * validates it against a given xml schema.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdValidatingInstanceReader : public XsdInstanceReader
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdValidatingInstanceReader> Ptr;

            /**
             * Creates a new validating instance reader that reads the data from
             * the given @p model.
             *
             * @param model The model the data shall be read from.
             * @param documentUri The uri of the document the model is from.
             * @param context The context that is used to report errors etc.
             */
            XsdValidatingInstanceReader(XsdValidatedXmlNodeModel *model, const QUrl &documentUri, const XsdSchemaContext::Ptr &context);

            /**
             * Adds a new @p schema to the pool of schemas that shall be used
             * for validation.
             * The schema is located at the given @p url.
             */
            void addSchema(const XsdSchema::Ptr &schema, const QUrl &url);

            /**
             * Reads and validates the instance document.
             */
            bool read();

        private:
            /**
             * Loads a schema with the given @p targetNamespace from the given @p location
             * and adds it to the pool of schemas that are used for validation.
             *
             * This method is used to load schemas defined in the xsi:schemaLocation or
             * xsi:noNamespaceSchemaLocation attributes in the instance document.
             */
            bool loadSchema(const QString &targetNamespace, const QUrl &location);

            /**
             * Reports an error via the report context.
             */
            void error(const QString &msg) const;

            /**
             * Validates the current element tag of the instance document.
             *
             * @param hasStateMachine Used to remember whether this element represents the start tag
             *                        of a complex type and therefor pushes a new state machine on the stack.
             * @param element Used to remember which element has been validated in this step.
             */
            bool validate(bool &hasStateMachine, XsdElement::Ptr &element);

            /**
             * Validates the current tag of the instance document against the given element @p declaration.
             *
             * @param declaration The element declaration to validate against.
             * @param hasStateMachine Used to remember whether this element represents the start tag
             *                        of a complex type and therefor pushes a new state machine on the stack.
             */
            bool validateElement(const XsdElement::Ptr &declaration, bool &hasStateMachine);

            /**
             * Validates the current tag of the instance document against the given @p type of the element @p declaration.
             *
             * @param declaration The element declaration to validate against.
             * @param type The type to validate against.
             * @param isNilled Defines whether the element is nilled by the instance document.
             * @param hasStateMachine Used to remember whether this element represents the start tag
             *                        of a complex type and therefor pushes a new state machine on the stack.
             *
             * @note The @p type can differ from the element @p declaration type if the instance document has defined
             *       it via xsi:type attribute.
             */
            bool validateElementType(const XsdElement::Ptr &declaration, const SchemaType::Ptr &type, bool isNilled, bool &hasStateMachine);

            /**
             * Validates the current tag of the instance document against the given simple @p type of the element @p declaration.
             *
             * @param declaration The element declaration to validate against.
             * @param type The type to validate against.
             * @param isNilled Defines whether the element is nilled by the instance document.
             *
             * @note The @p type can differ from the element @p declaration type if the instance document has defined
             *       it via xsi:type attribute.
             */
            bool validateElementSimpleType(const XsdElement::Ptr &declaration, const SchemaType::Ptr &type, bool isNilled);

            /**
             * Validates the current tag of the instance document against the given complex @p type of the element @p declaration.
             *
             * @param declaration The element declaration to validate against.
             * @param type The type to validate against.
             * @param isNilled Defines whether the element is nilled by the instance document.
             * @param hasStateMachine Used to remember whether this element represents the start tag
             *                        of a complex type and therefor pushes a new state machine on the stack.
             *
             * @note The @p type can differ from the element @p declaration type if the instance document has defined
             *       it via xsi:type attribute.
             */
            bool validateElementComplexType(const XsdElement::Ptr &declaration, const SchemaType::Ptr &type, bool isNilled, bool &hasStateMachine);

            /**
             * Validates the given @p value against the attribute use @p declaration.
             */
            bool validateAttribute(const XsdAttributeUse::Ptr &declaration, const QString &value);

            /**
             * Validates the given @p value against the attribute @p declaration.
             */
            bool validateAttribute(const XsdAttribute::Ptr &declaration, const QString &value);

            /**
             * Validates the given @p attributeName against the @p wildcard.
             */
            bool validateAttributeWildcard(const QXmlName &attributeName, const XsdWildcard::Ptr &wildcard);

            /**
             * Validates the identity constraints of an @p element.
             */
            bool validateIdentityConstraint(const XsdElement::Ptr &element, const QXmlItem &currentItem);

            /**
             * Validates the <em>unique</em> identity @p constraint of the @p element.
             */
            bool validateUniqueIdentityConstraint(const XsdElement::Ptr &element, const XsdIdentityConstraint::Ptr &constraint, const TargetNode::Set &qualifiedNodeSet);

            /**
             * Validates the <em>key</em> identity @p constraint of the @p element.
             */
            bool validateKeyIdentityConstraint(const XsdElement::Ptr &element, const XsdIdentityConstraint::Ptr &constraint, const TargetNode::Set &targetNodeSet, const TargetNode::Set &qualifiedNodeSet);

            /**
             * Validates the <em>keyref</em> identity @p constraint of the @p element.
             */
            bool validateKeyRefIdentityConstraint(const XsdElement::Ptr &element, const XsdIdentityConstraint::Ptr &constraint, const TargetNode::Set &qualifiedNodeSet);

            /**
             * Selects two sets of nodes that match the given identity @p constraint.
             *
             * @param element The element the identity constraint belongs to.
             * @param currentItem The current element that will be used as focus for the XQuery.
             * @param constraint The constraint (selector and fields) that describe the two sets.
             * @param targetNodeSet The target node set as defined by the schema specification.
             * @param qualifiedNodeSet The qualified node set as defined by the schema specification.
             */
            bool selectNodeSets(const XsdElement::Ptr &element, const QXmlItem &currentItem, const XsdIdentityConstraint::Ptr &constraint, TargetNode::Set &targetNodeSet, TargetNode::Set &qualifiedNodeSet);

            /**
             * Creates an QXmlQuery object with the defined @p namespaceBindings that has the @p contextNode as focus
             * and will execute @p query.
             */
            QXmlQuery createXQuery(const QList<QXmlName> &namespaceBindings, const QXmlItem &contextNode, const QString &query) const;

            /**
             * Returns the element declaration with the given @p name from the pool of all schemas.
             */
            XsdElement::Ptr elementByName(const QXmlName &name) const;

            /**
             * Returns the attribute declaration with the given @p name from the pool of all schemas.
             */
            XsdAttribute::Ptr attributeByName(const QXmlName &name) const;

            /**
             * Returns the type declaration with the given @p name from the pool of all schemas.
             */
            SchemaType::Ptr typeByName(const QXmlName &name) const;

            /**
             * Adds the ID/IDREF binding to the validated model and checks for duplicates.
             */
            void addIdIdRefBinding(const QString &id, const NamedSchemaComponent::Ptr &binding);

            /**
             * Helper method that reads an attribute of type xs:QName and does
             * syntax checking.
             */
            QString qNameAttribute(const QXmlName &attributeName);

            /**
             * Returns the xs:anyType that is used to build up the state machine.
             * We need that as the BuiltinTypes::xsAnyType is not a XsdComplexType.
             */
            XsdComplexType::Ptr anyType();

            /**
             * Helper method that creates a state machine for the given @p particle
             * and pushes it on the state machine stack.
             */
            void createAndPushStateMachine(const XsdParticle::Ptr &particle);

            typedef QHash<QUrl, QStringList> MergedSchemas;
            typedef QHashIterator<QUrl, QStringList> MergedSchemasIterator;

            XsdValidatedXmlNodeModel::Ptr               m_model;
            MergedSchemas                               m_mergedSchemas;
            XsdSchema::Ptr                              m_schema;
            const NamePool::Ptr                         m_namePool;
            const QXmlName                              m_xsiNilName;
            const QXmlName                              m_xsiTypeName;
            const QXmlName                              m_xsiSchemaLocationName;
            const QXmlName                              m_xsiNoNamespaceSchemaLocationName;

            QStack<XsdStateMachine<XsdTerm::Ptr> >      m_stateMachines;
            QUrl                                        m_documentUri;
            XsdComplexType::Ptr                         m_anyType;
            QSet<QString>                               m_processedNamespaces;
            QSet<QString>                               m_processedSchemaLocations;
            QSet<QString>                               m_idRefs;
            QHash<QXmlName, TargetNode::Set>            m_idcKeys;
            SchemaType::Ptr                             m_idRefsType;
    };
}

QT_END_NAMESPACE

#endif
