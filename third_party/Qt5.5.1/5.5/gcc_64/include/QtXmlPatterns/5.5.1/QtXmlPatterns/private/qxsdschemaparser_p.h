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

#ifndef Patternist_XsdSchemaParser_H
#define Patternist_XsdSchemaParser_H

#include <private/qnamespacesupport_p.h>
#include <private/qxsdalternative_p.h>
#include <private/qxsdattribute_p.h>
#include <private/qxsdattributegroup_p.h>
#include <private/qxsdattributeterm_p.h>
#include <private/qxsdcomplextype_p.h>
#include <private/qxsdelement_p.h>
#include <private/qxsdidcache_p.h>
#include <private/qxsdmodelgroup_p.h>
#include <private/qxsdnotation_p.h>
#include <private/qxsdsimpletype_p.h>
#include <private/qxsdschemacontext_p.h>
#include <private/qxsdschemaparsercontext_p.h>
#include <private/qxsdstatemachine_p.h>

#include <QtCore/QHash>
#include <QtCore/QSet>
#include <QtCore/QUrl>
#include <QtCore/QXmlStreamReader>
#include <QtXmlPatterns/QXmlNamePool>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Implements the parsing of XML schema file.
     *
     * This class parses a XML schema in XML presentation from an QIODevice
     * and returns object representation as XsdSchema.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchemaParser : public MaintainingReader<XsdSchemaToken, XsdTagScope::Type>
    {
        friend class ElementNamespaceHandler;
        friend class TagValidationHandler;

        public:
            enum ParserType
            {
                TopLevelParser,
                IncludeParser,
                ImportParser,
                RedefineParser
            };

            /**
             * Creates a new schema parser object.
             */
            XsdSchemaParser(const XsdSchemaContext::Ptr &context, const XsdSchemaParserContext::Ptr &parserContext, QIODevice *device);

            /**
             * Parses the XML schema file.
             *
             * @return @c true on success, @c false if the schema is somehow invalid.
             */
            bool parse(ParserType parserType = TopLevelParser);

            /**
             * Describes a set of namespace URIs
             */
            typedef QSet<QUrl> NamespaceSet;

            /**
             * Adds @p schemas to the list of already included schemas, so the parser
             * can detect multiple includes of the same schema.
             */
            void addIncludedSchemas(const NamespaceSet &schemas);

            /**
             * Sets which @p schemas have been included already, so the parser
             * can detect multiple includes of the same schema.
             */
            void setIncludedSchemas(const NamespaceSet &schemas);

            /**
             * Adds @p schemas to the list of already imported schemas, so the parser
             * can detect multiple imports of the same schema.
             */
            void addImportedSchemas(const NamespaceSet &schemas);

            /**
             * Sets which @p schemas have been imported already, so the parser
             * can detect circular imports.
             */
            void setImportedSchemas(const NamespaceSet &schemas);

            /**
             * Adds @p schemas to the list of already redefined schemas, so the parser
             * can detect multiple redefines of the same schema.
             */
            void addRedefinedSchemas(const NamespaceSet &schemas);

            /**
             * Sets which @p schemas have been redefined already, so the parser
             * can detect multiple redefines of the same schema.
             */
            void setRedefinedSchemas(const NamespaceSet &schemas);

            /**
             * Sets the target namespace of the schema to parse.
             */
            void setTargetNamespace(const QString &targetNamespace);

            /**
             * Sets the document URI of the schema to parse.
             */
            void setDocumentURI(const QUrl &uri);

            /**
             * Returns the document URI of the schema to parse.
             */
            QUrl documentURI() const;

            /**
             * Reimplemented from MaintainingReader, always returns @c false.
             */
            bool isAnyAttributeAllowed() const;

        private:
            /**
             * Used internally to report any kind of parsing error or
             * schema inconsistency.
             */
            virtual void error(const QString &msg);

            void attributeContentError(const char *attributeName, const char *elementName, const QString &value, const SchemaType::Ptr &type = SchemaType::Ptr());

            /**
             * Sets the target namespace of the schema to parse.
             */
            void setTargetNamespaceExtended(const QString &targetNamespace);

            /**
             * This method is called for parsing the top-level <em>schema</em> object.
             */
            void parseSchema(ParserType parserType);

            /**
             * This method is called for parsing any top-level <em>include</em> object.
             */
            void parseInclude();

            /**
             * This method is called for parsing any top-level <em>import</em> object.
             */
            void parseImport();

            /**
             * This method is called for parsing any top-level <em>redefine</em> object.
             */
            void parseRedefine();

            /**
             * This method is called for parsing any <em>annotation</em> object everywhere
             * in the schema.
             */
            XsdAnnotation::Ptr parseAnnotation();

            /**
             * This method is called for parsing an <em>appinfo</em> object as child of
             * an <em>annotation</em> object.
             */
            XsdApplicationInformation::Ptr parseAppInfo();

            /**
             * This method is called for parsing a <em>documentation</em> object as child of
             * an <em>annotation</em> object.
             */
            XsdDocumentation::Ptr parseDocumentation();

            /**
             * This method is called for parsing a <em>defaultOpenContent</em> object.
             */
            void parseDefaultOpenContent();

            /**
             * This method is called for parsing any top-level <em>simpleType</em> object.
             */
            XsdSimpleType::Ptr parseGlobalSimpleType();

            /**
             * This method is called for parsing any <em>simpleType</em> object as descendant
             * of an <em>element</em> or <em>complexType</em> object.
             */
            XsdSimpleType::Ptr parseLocalSimpleType();

            /**
             * This method is called for parsing a <em>restriction</em> object as child
             * of a <em>simpleType</em> object.
             */
            void parseSimpleRestriction(const XsdSimpleType::Ptr &ptr);

            /**
             * This method is called for parsing a <em>list</em> object as child
             * of a <em>simpleType</em> object.
             */
            void parseList(const XsdSimpleType::Ptr &ptr);

            /**
             * This method is called for parsing a <em>union</em> object as child
             * of a <em>simpleType</em> object.
             */
            void parseUnion(const XsdSimpleType::Ptr &ptr);

            /**
             * This method is called for parsing a <em>minExclusive</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMinExclusiveFacet();

            /**
             * This method is called for parsing a <em>minInclusive</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMinInclusiveFacet();

            /**
             * This method is called for parsing a <em>maxExclusive</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMaxExclusiveFacet();

            /**
             * This method is called for parsing a <em>maxInclusive</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMaxInclusiveFacet();

            /**
             * This method is called for parsing a <em>totalDigits</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseTotalDigitsFacet();

            /**
             * This method is called for parsing a <em>fractionDigits</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseFractionDigitsFacet();

            /**
             * This method is called for parsing a <em>length</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseLengthFacet();

            /**
             * This method is called for parsing a <em>minLength</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMinLengthFacet();

            /**
             * This method is called for parsing a <em>maxLength</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseMaxLengthFacet();

            /**
             * This method is called for parsing an <em>enumeration</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseEnumerationFacet();

            /**
             * This method is called for parsing a <em>whiteSpace</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseWhiteSpaceFacet();

            /**
             * This method is called for parsing a <em>pattern</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parsePatternFacet();

            /**
             * This method is called for parsing an <em>assertion</em> object as child
             * of a <em>restriction</em> object.
             */
            XsdFacet::Ptr parseAssertionFacet();

            /**
             * This method is called for parsing any top-level <em>complexType</em> object.
             */
            XsdComplexType::Ptr parseGlobalComplexType();

            /**
             * This method is called for parsing any <em>complexType</em> object as descendant
             * of an <em>element</em> object.
             */
            XsdComplexType::Ptr parseLocalComplexType();

            /**
             * This method resolves the content type of the @p complexType for the given
             * @p effectiveMixed value.
             */
            void resolveComplexContentType(const XsdComplexType::Ptr &complexType, bool effectiveMixed);

            /**
             * This method is called for parsing a <em>simpleContent</em> object as child
             * of a <em>complexType</em> object.
             */
            void parseSimpleContent(const XsdComplexType::Ptr &complexType);

            /**
             * This method is called for parsing a <em>restriction</em> object as child
             * of a <em>simpleContent</em> object.
             */
            void parseSimpleContentRestriction(const XsdComplexType::Ptr &complexType);

            /**
             * This method is called for parsing an <em>extension</em> object as child
             * of a <em>simpleContent</em> object.
             */
            void parseSimpleContentExtension(const XsdComplexType::Ptr &complexType);

            /**
             * This method is called for parsing a <em>complexContent</em> object as child
             * of a <em>complexType</em> object.
             *
             * @param complexType The complex type the complex content belongs to.
             * @param mixed The output parameter for the mixed value.
             */
            void parseComplexContent(const XsdComplexType::Ptr &complexType, bool *mixed);

            /**
             * This method is called for parsing a <em>restriction</em> object as child
             * of a <em>complexContent</em> object.
             */
            void parseComplexContentRestriction(const XsdComplexType::Ptr &complexType);

            /**
             * This method is called for parsing an <em>extension</em> object as child
             * of a <em>complexContent</em> object.
             */
            void parseComplexContentExtension(const XsdComplexType::Ptr &complexType);

            /**
             * This method is called for parsing an <em>assert</em> object as child
             * of a <em>complexType</em> or parsing a <em>assertion</em> facet object as
             * child of a <em>simpleType</em>.
             *
             * @param nodeName Either XsdSchemaToken::Assert or XsdSchemaToken::Assertion.
             * @param tag Either XsdTagScope::Assert or XsdTagScope::Assertion.
             */
            XsdAssertion::Ptr parseAssertion(const XsdSchemaToken::NodeName &nodeName, const XsdTagScope::Type &tag);

            /**
             * This method is called for parsing an <em>openContent</em> object.
             */
            XsdComplexType::OpenContent::Ptr parseOpenContent();

            /**
             * This method is called for parsing a top-level <em>group</em> object.
             */
            XsdModelGroup::Ptr parseNamedGroup();

            /**
             * This method is called for parsing a non-top-level <em>group</em> object
             * that contains a <em>ref</em> attribute.
             */
            XsdTerm::Ptr parseReferredGroup(const XsdParticle::Ptr &particle);

            /**
             * This method is called for parsing an <em>all</em> object as child
             * of a top-level <em>group</em> object.
             *
             * @param parent The schema component the <em>all</em> object is part of.
             */
            XsdModelGroup::Ptr parseAll(const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing an <em>all</em> object as descendant
             * of a <em>complexType</em> object.
             *
             * @param particle The particle the <em>all</em> object belongs to.
             * @param parent The schema component the <em>all</em> object is part of.
             */
            XsdModelGroup::Ptr parseLocalAll(const XsdParticle::Ptr &particle, const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a <em>choice</em> object as child
             * of a top-level <em>group</em> object.
             *
             * @param parent The schema component the <em>choice</em> object is part of.
             */
            XsdModelGroup::Ptr parseChoice(const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a <em>choice</em> object as descendant
             * of a <em>complexType</em> object or a <em>choice</em> object.
             *
             * @param particle The particle the <em>choice</em> object belongs to.
             * @param parent The schema component the <em>choice</em> object is part of.
             */
            XsdModelGroup::Ptr parseLocalChoice(const XsdParticle::Ptr &particle, const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a <em>sequence</em> object as child
             * of a top-level <em>group</em> object.
             *
             * @param parent The schema component the <em>sequence</em> object is part of.
             */
            XsdModelGroup::Ptr parseSequence(const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a <em>sequence</em> object as descendant
             * of a <em>complexType</em> object or a <em>sequence</em> object.
             *
             * @param particle The particle the <em>sequence</em> object belongs to.
             * @param parent The schema component the <em>sequence</em> object is part of.
             */
            XsdModelGroup::Ptr parseLocalSequence(const XsdParticle::Ptr &particle, const NamedSchemaComponent::Ptr &parent);

            /**
             * A helper method that parses the minOccurs and maxOccurs constraints for
             * the given @p particle that has the given @p tagName.
             */
            bool parseMinMaxConstraint(const XsdParticle::Ptr &particle, const char* tagName);

            /**
             * This method is called for parsing any top-level <em>attribute</em> object.
             */
            XsdAttribute::Ptr parseGlobalAttribute();

            /**
             * This method is called for parsing any non-top-level <em>attribute</em> object as a
             * descendant of a <em>complexType</em> object or an <em>attributeGroup</em> object.
             *
             * @param parent The parent component the <em>attribute</em> object is part of.
             */
            XsdAttributeUse::Ptr parseLocalAttribute(const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a top-level <em>attributeGroup</em> object.
             */
            XsdAttributeGroup::Ptr parseNamedAttributeGroup();

            /**
             * This method is called for parsing a non-top-level <em>attributeGroup</em> object
             * that contains a <em>ref</em> attribute.
             */
            XsdAttributeUse::Ptr parseReferredAttributeGroup();

            /**
             * This method is called for parsing any top-level <em>element</em> object.
             */
            XsdElement::Ptr parseGlobalElement();

            /**
             * This method is called for parsing any non-top-level <em>element</em> object as a
             * descendant of a <em>complexType</em> object or a <em>group</em> object.
             *
             * @param particle The particle the <em>element</em> object belongs to.
             * @param parent The parent component the <em>element</em> object is part of.
             */
            XsdTerm::Ptr parseLocalElement(const XsdParticle::Ptr &particle, const NamedSchemaComponent::Ptr &parent);

            /**
             * This method is called for parsing a <em>unique</em> object as child of an <em>element</em> object.
             */
            XsdIdentityConstraint::Ptr parseUnique();

            /**
             * This method is called for parsing a <em>key</em> object as child of an <em>element</em> object.
             */
            XsdIdentityConstraint::Ptr parseKey();

            /**
             * This method is called for parsing a <em>keyref</em> object as child of an <em>element</em> object.
             */
            XsdIdentityConstraint::Ptr parseKeyRef(const XsdElement::Ptr &element);

            /**
             * This method is called for parsing a <em>selector</em> object as child of an <em>unique</em> object,
             * <em>key</em> object or <em>keyref</em> object,
             *
             * @param ptr The identity constraint it belongs to.
             */
            void parseSelector(const XsdIdentityConstraint::Ptr &ptr);

            /**
             * This method is called for parsing a <em>field</em> object as child of an <em>unique</em> object,
             * <em>key</em> object or <em>keyref</em> object,
             *
             * @param ptr The identity constraint it belongs to.
             */
            void parseField(const XsdIdentityConstraint::Ptr &ptr);

            /**
             * This method is called for parsing an <em>alternative</em> object inside an <em>element</em> object.
             */
            XsdAlternative::Ptr parseAlternative();

            /**
             * This method is called for parsing a top-level <em>notation</em> object.
             */
            XsdNotation::Ptr parseNotation();

            /**
             * This method is called for parsing an <em>any</em> object somewhere in
             * the schema.
             *
             * @param particle The particle the <em>any</em> object belongs to.
             */
            XsdWildcard::Ptr parseAny(const XsdParticle::Ptr &particle);

            /**
             * This method is called for parsing an <em>anyAttribute</em> object somewhere in
             * the schema.
             */
            XsdWildcard::Ptr parseAnyAttribute();

            /**
             * This method is called for parsing unknown object as descendant of the <em>annotation</em> object.
             */
            void parseUnknownDocumentation();

            /**
             * This method is called for parsing unknown object in the schema.
             */
            void parseUnknown();

            /**
             * Returnes an source location for the current position.
             */
            QSourceLocation currentSourceLocation() const;

            /**
             * Converts a @p qualified name into a QXmlName @p name and does some error handling.
             */
            void convertName(const QString &qualified, NamespaceSupport::NameType type, QXmlName &name);

            /**
             * A helper method that reads in a 'name' attribute and checks it for syntactic errors.
             */
            inline QString readNameAttribute(const char *elementName);

            /**
             * A helper method that reads in an attribute that contains an QName and
             * checks it for syntactic errors.
             */
            inline QString readQNameAttribute(const QString &typeAttribute, const char *elementName);

            /**
             * A helper method that reads in a namespace attribute and checks for syntactic errors.
             */
            inline QString readNamespaceAttribute(const QString &attributeName, const char *elementName);

            /**
             * A helper method that reads the final attribute and does correct handling of schema default definitions.
             */
            inline SchemaType::DerivationConstraints readDerivationConstraintAttribute(const SchemaType::DerivationConstraints &allowedConstraints, const char *elementName);

            /**
             * A helper method that reads the block attribute and does correct handling of schema default definitions.
             */
            inline NamedSchemaComponent::BlockingConstraints readBlockingConstraintAttribute(const NamedSchemaComponent::BlockingConstraints &allowedConstraints, const char *elementName);

            /**
             * A helper method that reads all components for a xpath expression for the current scope.
             */
            XsdXPathExpression::Ptr readXPathExpression(const char *elementName);

            /**
             * Describes the type of XPath that is allowed by the readXPathAttribute method.
             */
            enum XPathType {
                XPath20,
                XPathSelector,
                XPathField
            };

            /**
             * A helper method that reads an attribute that represents a xpath query and does basic
             * validation.
             */
            QString readXPathAttribute(const QString &attributeName, XPathType type, const char *elementName);

            /**
             * A helper method that reads in an "id" attribute, checks it for syntactic errors
             * and tests whether a component with the same id has already been parsed.
             */
            inline void validateIdAttribute(const char *elementName);

            /**
             * Adds an @p element to the schema and checks for duplicated entries.
             */
            void addElement(const XsdElement::Ptr &element);

            /**
             * Adds an @p attribute to the schema and checks for duplicated entries.
             */
            void addAttribute(const XsdAttribute::Ptr &attribute);

            /**
             * Adds a @p type to the schema and checks for duplicated entries.
             */
            void addType(const SchemaType::Ptr &type);

            /**
             * Adds an anonymous @p type to the schema and checks for duplicated entries.
             */
            void addAnonymousType(const SchemaType::Ptr &type);

            /**
             * Adds an attribute @p group to the schema and checks for duplicated entries.
             */
            void addAttributeGroup(const XsdAttributeGroup::Ptr &group);

            /**
             * Adds an element @p group to the schema and checks for duplicated entries.
             */
            void addElementGroup(const XsdModelGroup::Ptr &group);

            /**
             * Adds a @p notation to the schema and checks for duplicated entries.
             */
            void addNotation(const XsdNotation::Ptr &notation);

            /**
             * Adds an identity @p constraint to the schema and checks for duplicated entries.
             */
            void addIdentityConstraint(const XsdIdentityConstraint::Ptr &constraint);

            /**
             * Adds the @p facet to the list of @p facets for @p type and checks for duplicates.
             */
            void addFacet(const XsdFacet::Ptr &facet, XsdFacet::Hash &facets, const SchemaType::Ptr &type);

            /**
             * Sets up the state machines for validating the right occurrence of xml elements.
             */
            void setupStateMachines();

            /**
             * Sets up a list of names of known builtin types.
             */
            void setupBuiltinTypeNames();

            /**
             * Checks whether the given @p tag is equal to the given @p token and
             * the given @p namespaceToken is the XML Schema namespace.
             */
            inline bool isSchemaTag(XsdSchemaToken::NodeName tag, XsdSchemaToken::NodeName token, XsdSchemaToken::NodeName namespaceToken) const;

            XsdSchemaContext::Ptr                                                m_context;
            XsdSchemaParserContext::Ptr                                          m_parserContext;
            NamePool::Ptr                                                        m_namePool;
            NamespaceSupport                                                     m_namespaceSupport;
            XsdSchemaResolver::Ptr                                               m_schemaResolver;
            XsdSchema::Ptr                                                       m_schema;

            QString                                                              m_targetNamespace;
            QString                                                              m_attributeFormDefault;
            QString                                                              m_elementFormDefault;
            QString                                                              m_blockDefault;
            QString                                                              m_finalDefault;
            QString                                                              m_xpathDefaultNamespace;
            QXmlName                                                             m_defaultAttributes;
            XsdComplexType::OpenContent::Ptr                                     m_defaultOpenContent;
            bool                                                                 m_defaultOpenContentAppliesToEmpty;

            NamespaceSet                                                         m_includedSchemas;
            NamespaceSet                                                         m_importedSchemas;
            NamespaceSet                                                         m_redefinedSchemas;
            QUrl                                                                 m_documentURI;
            XsdIdCache::Ptr                                                      m_idCache;
            QHash<XsdTagScope::Type, XsdStateMachine<XsdSchemaToken::NodeName> > m_stateMachines;
            ComponentLocationHash                                                m_componentLocationHash;
            QSet<QXmlName>                                                       m_builtinTypeNames;
    };
}

QT_END_NAMESPACE

#endif
