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

#ifndef Patternist_XsdSchemaResolver_H
#define Patternist_XsdSchemaResolver_H

#include <private/qnamespacesupport_p.h>
#include <private/qschematype_p.h>
#include <private/qschematypefactory_p.h>
#include <private/qxsdalternative_p.h>
#include <private/qxsdattribute_p.h>
#include <private/qxsdattributegroup_p.h>
#include <private/qxsdelement_p.h>
#include <private/qxsdmodelgroup_p.h>
#include <private/qxsdnotation_p.h>
#include <private/qxsdreference_p.h>
#include <private/qxsdschema_p.h>
#include <private/qxsdschemachecker_p.h>
#include <private/qxsdsimpletype_p.h>

#include <QtCore/QExplicitlySharedDataPointer>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class XsdSchemaContext;
    class XsdSchemaParserContext;

    /**
     * @short Encapsulates the resolving of type/element references in a schema after parsing has finished.
     *
     * This class collects task for resolving types or element references. After the parsing has finished,
     * one can start the resolve process by calling resolve().
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchemaResolver : public QSharedData
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdSchemaResolver> Ptr;

            /**
             * Creates a new schema resolver.
             *
             * @param context The schema context used for error reporting etc..
             * @param parserContext The schema parser context where all objects to resolve belong to.
             */
            XsdSchemaResolver(const QExplicitlySharedDataPointer<XsdSchemaContext> &context, const XsdSchemaParserContext *parserContext);

            /**
             * Destroys the schema resolver.
             */
            ~XsdSchemaResolver();

            /**
             * Starts the resolve process.
             */
            void resolve();

            /**
             * Adds a resolve task for key references.
             *
             * The resolver will try to set the referencedKey property of @p keyRef to the <em>key</em> or <em>unique</em> object
             * of @p element that has the given @p name.
             */
            void addKeyReference(const XsdElement::Ptr &element, const XsdIdentityConstraint::Ptr &keyRef, const QXmlName &name, const QSourceLocation &location);

            /**
             * Adds a resolve task for the base type of restriction of a simple type.
             *
             * The resolver will set the base type of @p simpleType to the type named by @p baseName.
             */
            void addSimpleRestrictionBase(const XsdSimpleType::Ptr &simpleType, const QXmlName &baseName, const QSourceLocation &location);

            /**
             * Removes the resolve task for the base type of restriction of the simple @p type.
             */
            void removeSimpleRestrictionBase(const XsdSimpleType::Ptr &type);

            /**
             * Adds a resolve task for the list type of a simple type.
             *
             * The resolver will set the itemType property of @p simpleType to the type named by @p typeName.
             */
            void addSimpleListType(const XsdSimpleType::Ptr &simpleType, const QXmlName &typeName, const QSourceLocation &location);

            /**
             * Adds a resolve task for the member types of a simple type.
             *
             * The resolver will set the memberTypes property of @p simpleType to the types named by @p typeNames.
             */
            void addSimpleUnionTypes(const XsdSimpleType::Ptr &simpleType, const QList<QXmlName> &typeNames, const QSourceLocation &location);

            /**
             * Adds a resolve task for the type of an element.
             *
             * The resolver will set the type of the @p element to the type named by @p typeName.
             */
            void addElementType(const XsdElement::Ptr &element, const QXmlName &typeName, const QSourceLocation &location);

            /**
             * Adds a resolve task for the base type of a complex type.
             *
             * The resolver will set the base type of @p complexType to the type named by @p baseName.
             */
            void addComplexBaseType(const XsdComplexType::Ptr &complexType, const QXmlName &baseName, const QSourceLocation &location, const XsdFacet::Hash &facets = XsdFacet::Hash());

            /**
             * Removes the resolve task for the base type of the complex @p type.
             */
            void removeComplexBaseType(const XsdComplexType::Ptr &type);

            /**
             * Adds a resolve task for the content type of a complex type.
             *
             * The resolver will set the content type properties for @p complexType based on the
             * given explicit @p content and effective @p mixed value.
             */
            void addComplexContentType(const XsdComplexType::Ptr &complexType, const XsdParticle::Ptr &content, bool mixed);

            /**
             * Adds a resolve task for the type of an attribute.
             *
             * The resolver will set the type of the @p attribute to the type named by @p typeName.
             */
            void addAttributeType(const XsdAttribute::Ptr &attribute, const QXmlName &typeName, const QSourceLocation &location);

            /**
             * Adds a resolve task for the type of an alternative.
             *
             * The resolver will set the type of the @p alternative to the type named by @p typeName.
             */
            void addAlternativeType(const XsdAlternative::Ptr &alternative, const QXmlName &typeName, const QSourceLocation &location);

            /**
             * Adds a resolve task for the type of an alternative.
             *
             * The resolver will set the type of the @p alternative to the type of the @p element after
             * the type of the @p element has been resolved.
             */
            void addAlternativeType(const XsdAlternative::Ptr &alternative, const XsdElement::Ptr &element);

            /**
             * Adds a resolve task for the substituion group affiliations of an element.
             *
             * The resolver will set the substitution group affiliations of the @p element to the
             * top-level element named by @p elementNames.
             */
            void addSubstitutionGroupAffiliation(const XsdElement::Ptr &element, const QList<QXmlName> &elementName, const QSourceLocation &location);

            /**
             * Adds a resolve task for an element that has no type specified, only a substitution group
             * affiliation.
             *
             * The resolver will set the type of the substitution group affiliation as type for the element.
             */
            void addSubstitutionGroupType(const XsdElement::Ptr &element);

            /**
             * Adds the component location hash, so the resolver is able to report meaning full
             * error messages.
             */
            void addComponentLocationHash(const QHash<NamedSchemaComponent::Ptr, QSourceLocation> &hash);

            /**
             * Add a resolve task for enumeration facet values.
             *
             * In case the enumeration is of type QName or NOTATION, we have to resolve the QName later,
             * so we store the namespace bindings together with the facet value here and resolve it as soon as
             * we have all type information available.
             */
            void addEnumerationFacetValue(const AtomicValue::Ptr &facetValue, const NamespaceSupport &namespaceSupport);

            /**
             * Add a check job for redefined groups.
             *
             * When an element group is redefined, we have to check whether the redefined group is a valid
             * restriction of the group it redefines. As we need all type information for that, we keep them
             * here for later checking.
             */
            void addRedefinedGroups(const XsdModelGroup::Ptr &redefinedGroup, const XsdModelGroup::Ptr &group);

            /**
             * Add a check job for redefined attribute groups.
             *
             * When an attribute group is redefined, we have to check whether the redefined group is a valid
             * restriction of the group it redefines. As we need all type information for that, we keep them
             * here for later checking.
             */
            void addRedefinedAttributeGroups(const XsdAttributeGroup::Ptr &redefinedGroup, const XsdAttributeGroup::Ptr &group);

            /**
             * Adds a check for nested <em>all</em> groups.
             */
            void addAllGroupCheck(const XsdReference::Ptr &reference);

            /**
             * Copies the data to resolve to an @p other resolver.
             *
             * @note That functionality is only used by the redefine algorithm in the XsdSchemaParser.
             */
            void copyDataTo(const XsdSchemaResolver::Ptr &other) const;

            /**
             * Returns the to resolve base type name for the given @p type.
             *
             * @note That functionality is only used by the redefine algorithm in the XsdSchemaParser.
             */
            QXmlName baseTypeNameOfType(const SchemaType::Ptr &type) const;

            /**
             * Returns the to resolve type name for the given @p attribute.
             *
             * @note That functionality is only used by the redefine algorithm in the XsdSchemaParser.
             */
            QXmlName typeNameOfAttribute(const XsdAttribute::Ptr &attribute) const;

            /**
             * Sets the defaultOpenContent object from the schema parser.
             */
            void setDefaultOpenContent(const XsdComplexType::OpenContent::Ptr &openContent, bool appliesToEmpty);

        private:
            /**
             * Resolves key references.
             */
            void resolveKeyReferences();

            /**
             * Resolves the base types of simple types derived by restriction.
             */
            void resolveSimpleRestrictionBaseTypes();

            /**
             * Resolves the other properties except the base type
             * of all simple restrictions.
             */
            void resolveSimpleRestrictions();

            /**
             * Resolves the other properties except the base type
             * of the given simple restriction.
             *
             * @param simpleType The restricted type to resolve.
             * @param visitedTypes A set of already resolved types, used for termination of recursion.
             */
            void resolveSimpleRestrictions(const XsdSimpleType::Ptr &simpleType, QSet<XsdSimpleType::Ptr> &visitedTypes);

            /**
             * Resolves the item type property of simple types derived by list.
             */
            void resolveSimpleListType();

            /**
             * Resolves the member types property of simple types derived by union.
             */
            void resolveSimpleUnionTypes();

            /**
             * Resolves element types.
             */
            void resolveElementTypes();

            /**
             * Resolves base type of complex types.
             */
            void resolveComplexBaseTypes();

            /**
             * Resolves the simple content model of a complex type
             * depending on its base type.
             */
            void resolveSimpleContentComplexTypes();

            /**
             * Resolves the complex content model of a complex type
             * depending on its base type.
             */
            void resolveComplexContentComplexTypes();

            /**
             * Resolves the simple content model of a complex type
             * depending on its base type.
             *
             * @param complexType The complex type to resolve.
             * @param visitedTypes A set of already resolved types, used for termination of recursion.
             */
            void resolveSimpleContentComplexTypes(const XsdComplexType::Ptr &complexType, QSet<XsdComplexType::Ptr> &visitedTypes);

            /**
             * Resolves the complex content model of a complex type
             * depending on its base type.
             *
             * @param complexType The complex type to resolve.
             * @param visitedTypes A set of already resolved types, used for termination of recursion.
             */
            void resolveComplexContentComplexTypes(const XsdComplexType::Ptr &complexType, QSet<XsdComplexType::Ptr> &visitedTypes);

            /**
             * Resolves attribute types.
             */
            void resolveAttributeTypes();

            /**
             * Resolves alternative types.
             */
            void resolveAlternativeTypes();

            /**
             * Resolves substitution group affiliations.
             */
            void resolveSubstitutionGroupAffiliations();

            /**
             * Resolves substitution groups.
             */
            void resolveSubstitutionGroups();

            /**
             * Resolves all XsdReferences in the schema by their corresponding XsdElement or XsdModelGroup terms.
             */
            void resolveTermReferences();

            /**
             * Resolves all XsdReferences in the @p particle recursive by their corresponding XsdElement or XsdModelGroup terms.
             */
            void resolveTermReference(const XsdParticle::Ptr &particle, QSet<QXmlName> visitedGroups);

            /**
             * Resolves all XsdAttributeReferences in the schema by their corresponding XsdAttributeUse objects.
             */
            void resolveAttributeTermReferences();

            /**
             * Resolves all XsdAttributeReferences in the list of @p attributeUses by their corresponding XsdAttributeUse objects.
             */
            XsdAttributeUse::List resolveAttributeTermReferences(const XsdAttributeUse::List &attributeUses, XsdWildcard::Ptr &wildcard, QSet<QXmlName> visitedAttributeGroups);

            /**
             * Resolves the attribute inheritance of complex types.
             *
             * @note This method must be called after all base types have been resolved.
             */
            void resolveAttributeInheritance();

            /**
             * Resolves the attribute inheritance of the given complex types.
             *
             * @param complexType The complex type to resolve.
             * @param visitedTypes A set of already resolved types, used for termination of recursion.
             *
             * @note This method must be called after all base types have been resolved.
             */
            void resolveAttributeInheritance(const XsdComplexType::Ptr &complexType, QSet<XsdComplexType::Ptr> &visitedTypes);

            /**
             * Resolves the enumeration facet values for QName and NOTATION based facets.
             */
            void resolveEnumerationFacetValues();

            /**
             * Returns the source location of the given schema @p component or a dummy
             * source location if the component is not found in the component location hash.
             */
            QSourceLocation sourceLocation(const NamedSchemaComponent::Ptr component) const;

            /**
             * Returns the facets that are marked for the given complex @p type with a simple
             * type restriction.
             */
            XsdFacet::Hash complexTypeFacets(const XsdComplexType::Ptr &complexType) const;

            /**
             * Finds the primitive type for the given simple @p type.
             *
             * The type is found by walking up the inheritance tree, until one of the builtin
             * primitive type definitions is reached.
             */
            AnySimpleType::Ptr findPrimitiveType(const AnySimpleType::Ptr &type, QSet<AnySimpleType::Ptr> &visitedTypes);

            /**
             * Checks the redefined groups.
             */
            void checkRedefinedGroups();

            /**
             * Checks the redefined attribute groups.
             */
            void checkRedefinedAttributeGroups();

            class KeyReference
            {
                public:
                    XsdElement::Ptr element;
                    XsdIdentityConstraint::Ptr keyRef;
                    QXmlName reference;
                    QSourceLocation location;
            };

            class SimpleRestrictionBase
            {
                public:
                    XsdSimpleType::Ptr simpleType;
                    QXmlName baseName;
                    QSourceLocation location;
            };

            class SimpleListType
            {
                public:
                    XsdSimpleType::Ptr simpleType;
                    QXmlName typeName;
                    QSourceLocation location;
            };

            class SimpleUnionType
            {
                public:
                    XsdSimpleType::Ptr simpleType;
                    QList<QXmlName> typeNames;
                    QSourceLocation location;
            };

            class ElementType
            {
                public:
                    XsdElement::Ptr element;
                    QXmlName typeName;
                    QSourceLocation location;
            };

            class ComplexBaseType
            {
                public:
                    XsdComplexType::Ptr complexType;
                    QXmlName baseName;
                    QSourceLocation location;
                    XsdFacet::Hash facets;
            };

            class ComplexContentType
            {
                public:
                    XsdComplexType::Ptr complexType;
                    XsdParticle::Ptr explicitContent;
                    bool effectiveMixed;
            };

            class AttributeType
            {
                public:
                    XsdAttribute::Ptr attribute;
                    QXmlName typeName;
                    QSourceLocation location;
            };

            class AlternativeType
            {
                public:
                    XsdAlternative::Ptr alternative;
                    QXmlName typeName;
                    QSourceLocation location;
            };

            class AlternativeTypeElement
            {
                public:
                    XsdAlternative::Ptr alternative;
                    XsdElement::Ptr element;
            };

            class SubstitutionGroupAffiliation
            {
                public:
                    XsdElement::Ptr element;
                    QList<QXmlName> elementNames;
                    QSourceLocation location;
            };

            class RedefinedGroups
            {
                public:
                    XsdModelGroup::Ptr redefinedGroup;
                    XsdModelGroup::Ptr group;
            };

            class RedefinedAttributeGroups
            {
                public:
                    XsdAttributeGroup::Ptr redefinedGroup;
                    XsdAttributeGroup::Ptr group;
            };

            QVector<KeyReference>                                m_keyReferences;
            QVector<SimpleRestrictionBase>                       m_simpleRestrictionBases;
            QVector<SimpleListType>                              m_simpleListTypes;
            QVector<SimpleUnionType>                             m_simpleUnionTypes;
            QVector<ElementType>                                 m_elementTypes;
            QVector<ComplexBaseType>                             m_complexBaseTypes;
            QVector<ComplexContentType>                          m_complexContentTypes;
            QVector<AttributeType>                               m_attributeTypes;
            QVector<AlternativeType>                             m_alternativeTypes;
            QVector<AlternativeTypeElement>                      m_alternativeTypeElements;
            QVector<SubstitutionGroupAffiliation>                m_substitutionGroupAffiliations;
            QVector<XsdElement::Ptr>                             m_substitutionGroupTypes;
            QVector<RedefinedGroups>                             m_redefinedGroups;
            QVector<RedefinedAttributeGroups>                    m_redefinedAttributeGroups;
            QHash<AtomicValue::Ptr, NamespaceSupport>            m_enumerationFacetValues;
            QSet<XsdReference::Ptr>                              m_allGroups;

            QExplicitlySharedDataPointer<XsdSchemaContext>       m_context;
            QExplicitlySharedDataPointer<XsdSchemaChecker>       m_checker;
            NamePool::Ptr                                        m_namePool;
            XsdSchema::Ptr                                       m_schema;
            QHash<NamedSchemaComponent::Ptr, QSourceLocation>    m_componentLocationHash;
            XsdComplexType::OpenContent::Ptr                     m_defaultOpenContent;
            bool                                                 m_defaultOpenContentAppliesToEmpty;
            SchemaType::List                                     m_predefinedSchemaTypes;
    };
}

QT_END_NAMESPACE

#endif
