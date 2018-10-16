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

#ifndef Patternist_XsdSchemaChecker_H
#define Patternist_XsdSchemaChecker_H

#include <private/qschematype_p.h>
#include <private/qxsdattribute_p.h>
#include <private/qxsdattributegroup_p.h>
#include <private/qxsdelement_p.h>
#include <private/qxsdmodelgroup_p.h>
#include <private/qxsdnotation_p.h>
#include <private/qxsdschema_p.h>
#include <private/qxsdsimpletype_p.h>

#include <QtCore/QExplicitlySharedDataPointer>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class XsdSchemaContext;
    class XsdSchemaParserContext;

    /**
     * @short Encapsulates the checking of schema valitity after reference resolving has finished.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchemaChecker : public QSharedData
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdSchemaChecker> Ptr;

            /**
             * Creates a new schema checker.
             *
             * @param context The context that is used for customization.
             * @param parserContext The context that contains all the data structures.
             */
            XsdSchemaChecker(const QExplicitlySharedDataPointer<XsdSchemaContext> &context, const XsdSchemaParserContext *parserContext);

            /**
             * Destroys the schema checker.
             */
            ~XsdSchemaChecker();

            /**
             * Starts a basic check process.
             *
             * This check only validates the basic super type inheritance
             * of simple and complex types.
             */
            void basicCheck();

            /**
             * Starts the real check process.
             */
            void check();

            /**
             * Checks the constraining facets of all global and anonymous simple types for validity.
             */
            void checkConstrainingFacets();

            /**
             * Adds the component location hash, so the checker is able to report meaning full
             * error messages.
             */
            void addComponentLocationHash(const QHash<NamedSchemaComponent::Ptr, QSourceLocation> &hash);

        private:
            void checkSimpleRestrictionBaseType();

            /**
             * Checks that no simple or complex type inherits itself.
             */
            void checkBasicCircularInheritances();

            /**
             * Checks the advanced circular inheritance.
             */
            void checkCircularInheritances();

            /**
             * Checks for inheritance restrictions given by final or finalDefault
             * attributes.
             */
            void checkInheritanceRestrictions();

            /**
             * Checks for various constraints for simple types defined by schema.
             */
            void checkBasicSimpleTypeConstraints();
            void checkSimpleTypeConstraints();

            /**
             * Checks for various constraints for complex types defined by schema.
             */
            void checkBasicComplexTypeConstraints();
            void checkComplexTypeConstraints();

            /**
             * Checks for list and union derivation restrictions given by final or finalDefault
             * attributes.
             */
            void checkSimpleDerivationRestrictions();

            /**
             * Checks the set of constraining @p facets that belongs to @p simpleType for validity.
             */
            void checkConstrainingFacets(const XsdFacet::Hash &facets, const XsdSimpleType::Ptr &simpleType);

            /**
             * Checks for duplicated attribute uses (attributes with the same name) inside a complex type.
             */
            void checkDuplicatedAttributeUses();

            /**
             * Check the element constraints.
             */
            void checkElementConstraints();

            /**
             * Check the attribute constraints.
             */
            void checkAttributeConstraints();

            /**
             * Check the attribute use constraints.
             */
            void checkAttributeUseConstraints();

            /**
             * A map used to find duplicated elements inside a model group.
             */
            typedef QHash<QXmlName, SchemaType::Ptr> DuplicatedElementMap;

            /**
             * A map used to find duplicated wildcards inside a model group.
             */
            typedef QHash<XsdWildcard::NamespaceConstraint::Variety, XsdWildcard::Ptr> DuplicatedWildcardMap;

            /**
             * Check for duplicated elements and element wildcards in all complex type particles.
             */
            void checkElementDuplicates();

            /**
             * Check for duplicated elements and element wildcards in the given @p particle.
             *
             * @param particle The particle to check.
             * @param elementMap A map to find the duplicated elements.
             * @param wildcardMap A map to find the duplicated element wildcards.
             */
            void checkElementDuplicates(const XsdParticle::Ptr &particle, DuplicatedElementMap &elementMap, DuplicatedWildcardMap &wildcardMap);

            /**
             * Setup fast lookup list for allowed facets of atomic simple types.
             */
            void setupAllowedAtomicFacets();

            /**
             * Returns the source location of the given schema @p component or a dummy
             * source location if the component is not found in the component location hash.
             */
            QSourceLocation sourceLocation(const NamedSchemaComponent::Ptr &component) const;

            /**
             * Returns the source location of the given schema @p type or a dummy
             * source location if the type is not found in the component location hash.
             */
            QSourceLocation sourceLocationForType(const SchemaType::Ptr &type) const;

            /**
             * Checks that the string @p value is valid according the value space of @p type
             * for the given @p component.
             */
            bool isValidValue(const QString &value, const AnySimpleType::Ptr &type, QString &errorMsg) const;

            /**
             * Returns the list of facets for the given @p type.
             */
            XsdFacet::Hash facetsForType(const SchemaType::Ptr &type) const;

            /**
             * Returns whether the given @p list of attribute uses contains two (or more) attribute
             * uses that point to attributes with the same name. @p conflictingAttribute
             * will contain the conflicting attribute in that case.
             */
            bool hasDuplicatedAttributeUses(const XsdAttributeUse::List &list, XsdAttribute::Ptr &conflictingAttribute) const;

            /**
             * Returns whether the given @p list of attribute uses contains two (or more) attribute
             * uses that have a type inherited by xs:ID.
             */
            bool hasMultipleIDAttributeUses(const XsdAttributeUse::List &list) const;

            /**
             * Returns whether the given @p list of attribute uses contains an attribute
             * uses that has a type inherited by xs:ID with a value constraint. @p conflictingAttribute
             * will contain the conflicting attribute in that case.
             */
            bool hasConstraintIDAttributeUse(const XsdAttributeUse::List &list, XsdAttribute::Ptr &conflictingAttribute) const;

            /**
             * Checks whether the @p particle equals the @p otherParticle recursively.
             */
            bool particleEqualsRecursively(const XsdParticle::Ptr &particle, const XsdParticle::Ptr &otherParticle) const;

            /**
             * Checks whether the @p extension particle is a valid extension of the @p base particle.
             */
            bool isValidParticleExtension(const XsdParticle::Ptr &extension, const XsdParticle::Ptr &base) const;

            /**
             * Checks whether the @p sequence of elements is accepted by the given @p particle.
             */
            bool elementSequenceAccepted(const XsdModelGroup::Ptr &sequence, const XsdParticle::Ptr &particle) const;

            QExplicitlySharedDataPointer<XsdSchemaContext>       m_context;
            NamePool::Ptr                                        m_namePool;
            XsdSchema::Ptr                                       m_schema;
            QHash<QXmlName, QSet<XsdFacet::Type> >               m_allowedAtomicFacets;
            QHash<NamedSchemaComponent::Ptr, QSourceLocation>    m_componentLocationHash;
    };
}

QT_END_NAMESPACE

#endif
