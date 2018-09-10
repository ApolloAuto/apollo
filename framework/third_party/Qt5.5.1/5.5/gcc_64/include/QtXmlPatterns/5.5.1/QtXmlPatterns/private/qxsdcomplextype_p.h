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

#ifndef Patternist_XsdComplexType_H
#define Patternist_XsdComplexType_H

#include <private/qanytype_p.h>
#include <private/qxsdassertion_p.h>
#include <private/qxsdattributeuse_p.h>
#include <private/qxsdparticle_p.h>
#include <private/qxsdsimpletype_p.h>
#include <private/qxsduserschematype_p.h>
#include <private/qxsdwildcard_p.h>

#include <QtCore/QSet>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a XSD complexType object.
     *
     * This class represents the <em>complexType</em> object of a XML schema as described
     * <a href="http://www.w3.org/TR/xmlschema11-1/#Complex_Type_Definitions">here</a>.
     *
     * It contains information from either a top-level complex type declaration (as child of a <em>schema</em> object)
     * or a local complex type declaration (as descendant of an <em>element</em> object).
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSComplexType">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdComplexType : public XsdUserSchemaType<AnyType>
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdComplexType> Ptr;

            /**
             * @short Describes the open content object of a complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ct-open_content">Open Content Definition</a>
             */
            class OpenContent : public QSharedData, public XsdAnnotated
            {
                public:
                    typedef QExplicitlySharedDataPointer<OpenContent> Ptr;

                    /**
                     * Describes the mode of the open content.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#oc-mode">Mode Definition</a>
                     */
                    enum Mode
                    {
                        None,
                        Interleave,
                        Suffix
                    };

                    /**
                     * Sets the @p mode of the open content.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#oc-mode">Mode Definition</a>
                     */
                    void setMode(Mode mode);

                    /**
                     * Returns the mode of the open content.
                     */
                    Mode mode() const;

                    /**
                     * Sets the @p wildcard of the open content.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#oc-wildcard">Wildcard Definition</a>
                     */
                    void setWildcard(const XsdWildcard::Ptr &wildcard);

                    /**
                     * Returns the wildcard of the open content.
                     */
                    XsdWildcard::Ptr wildcard() const;

                private:
                    Mode             m_mode;
                    XsdWildcard::Ptr m_wildcard;
            };

            /**
             * @short Describes the content type of a complex type.
             */
            class ContentType : public QSharedData
            {
                public:
                    typedef QExplicitlySharedDataPointer<ContentType> Ptr;

                    /**
                     * Describes the variety of the content type.
                     */
                    enum Variety
                    {
                        Empty = 0,    ///< The complex type has no further content.
                        Simple,       ///< The complex type has only simple type content (e.g. text, number etc.)
                        ElementOnly,  ///< The complex type has further elements or attributes but no text as content.
                        Mixed         ///< The complex type has further elements or attributes and text as content.
                    };

                    /**
                     * Sets the @p variety of the content type.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ct-variety">Variety Definition</a>
                     */
                    void setVariety(Variety variety);

                    /**
                     * Returns the variety of the content type.
                     */
                    Variety variety() const;

                    /**
                     * Sets the @p particle object of the content type.
                     *
                     * The content type has only a particle object if
                     * its variety is ElementOnly or Mixed.
                     *
                     * @see XsdParticle
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ct-particle">Particle Declaration</a>
                     */
                    void setParticle(const XsdParticle::Ptr &particle);

                    /**
                     * Returns the particle object of the content type,
                     * or an empty pointer if its variety is neither
                     * ElementOnly nor Mixed.
                     */
                    XsdParticle::Ptr particle() const;

                    /**
                     * Sets the open @p content object of the content type.
                     *
                     * The content type has only an open content object if
                     * its variety is ElementOnly or Mixed.
                     *
                     * @see OpenContent
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ct-open_content">Open Content Declaration</a>
                     */
                    void setOpenContent(const OpenContent::Ptr &content);

                    /**
                     * Returns the open content object of the content type,
                     * or an empty pointer if its variety is neither
                     * ElementOnly nor Mixed.
                     */
                    OpenContent::Ptr openContent() const;

                    /**
                     * Sets the simple @p type object of the content type.
                     *
                     * The content type has only a simple type object if
                     * its variety is Simple.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ct-simple_type_definition">Simple Type Definition</a>
                     */
                    void setSimpleType(const AnySimpleType::Ptr &type);

                    /**
                     * Returns the simple type object of the content type,
                     * or an empty pointer if its variety is not Simple.
                     */
                    AnySimpleType::Ptr simpleType() const;

                private:
                    Variety            m_variety;
                    XsdParticle::Ptr   m_particle;
                    OpenContent::Ptr   m_openContent;
                    XsdSimpleType::Ptr m_simpleType;
            };


            /**
             * Creates a complex type object with empty content.
             */
            XsdComplexType();

            /**
             * Destroys the complex type object.
             */
            ~XsdComplexType() {};

            /**
             * Returns the display name of the complex type.
             *
             * The display name can be used to show the type name
             * to the user.
             *
             * @param namePool The name pool where the type name is stored in.
             */
            virtual QString displayName(const NamePool::Ptr &namePool) const;

            /**
             * Sets the base type of the complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-base_type_definition">Base Type Definition</a>
             */
            void setWxsSuperType(const SchemaType::Ptr &type);

            /**
             * Returns the base type of the complex type.
             */
            virtual SchemaType::Ptr wxsSuperType() const;

            /**
             * Sets the context @p component of the complex type.
             *
             * The component is either an element declaration or a complex type definition.
             */
            void setContext(const NamedSchemaComponent::Ptr &component);

            /**
             * Returns the context component of the complex type.
             */
            NamedSchemaComponent::Ptr context() const;

            /**
             * Sets the derivation @p method of the complex type.
             *
             * The derivation method depends on whether the complex
             * type object has an extension or restriction object as child.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-derivation_method">Derivation Method Definition</a>
             * @see DerivationMethod
             */
            void setDerivationMethod(DerivationMethod method);

            /**
             * Returns the derivation method of the complex type.
             */
            virtual DerivationMethod derivationMethod() const;

            /**
             * Sets whether the complex type is @p abstract.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-abstract">Abstract Definition</a>
             */
            void setIsAbstract(bool abstract);

            /**
             * Returns whether the complex type is abstract.
             */
            virtual bool isAbstract() const;

            /**
             * Sets the list of all attribute @p uses of the complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-attribute_uses">Attribute Uses Declaration</a>
             */
            void setAttributeUses(const XsdAttributeUse::List &uses);

            /**
             * Adds a new attribute @p use to the complex type.
             */
            void addAttributeUse(const XsdAttributeUse::Ptr &use);

            /**
             * Returns the list of all attribute uses of the complex type.
             */
            XsdAttributeUse::List attributeUses() const;

            /**
             * Sets the attribute @p wildcard of the complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-attribute_wildcard">Attribute Wildcard Declaration</a>
             */
            void setAttributeWildcard(const XsdWildcard::Ptr &wildcard);

            /**
             * Returns the attribute wildcard of the complex type.
             */
            XsdWildcard::Ptr attributeWildcard() const;

            /**
             * Always returns SchemaType::ComplexType
             */
            virtual TypeCategory category() const;

            /**
             * Sets the content @p type of the complex type.
             *
             * @see ContentType
             */
            void setContentType(const ContentType::Ptr &type);

            /**
             * Returns the content type of the complex type.
             */
            ContentType::Ptr contentType() const;

            /**
             * Sets the prohibited @p substitutions of the complex type.
             *
             * Only ExtensionConstraint and RestrictionConstraint are allowed.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-prohibited_substitutions">Prohibited Substitutions Definition</a>
             */
            void setProhibitedSubstitutions(const BlockingConstraints &substitutions);

            /**
             * Returns the prohibited substitutions of the complex type.
             */
            BlockingConstraints prohibitedSubstitutions() const;

            /**
             * Sets the @p assertions of the complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-assertions">Assertions Definition</a>
             */
            void setAssertions(const XsdAssertion::List &assertions);

            /**
             * Adds an @p assertion to the complex type.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ctd-assertions">Assertions Definition</a>
             */
            void addAssertion(const XsdAssertion::Ptr &assertion);

            /**
             * Returns the assertions of the complex type.
             */
            XsdAssertion::List assertions() const;

            /**
             * Always returns @c true.
             */
            virtual bool isDefinedBySchema() const;

        private:
            SchemaType::Ptr           m_superType;
            NamedSchemaComponent::Ptr m_context;
            DerivationMethod          m_derivationMethod;
            bool                      m_isAbstract;
            XsdAttributeUse::List     m_attributeUses;
            XsdWildcard::Ptr          m_attributeWildcard;
            ContentType::Ptr          m_contentType;
            BlockingConstraints       m_prohibitedSubstitutions;
            XsdAssertion::List        m_assertions;
    };
}

QT_END_NAMESPACE

#endif
