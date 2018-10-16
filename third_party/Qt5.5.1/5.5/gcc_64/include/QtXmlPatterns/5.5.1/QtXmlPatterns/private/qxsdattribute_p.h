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

#ifndef Patternist_XsdAttribute_H
#define Patternist_XsdAttribute_H

#include <private/qanysimpletype_p.h>
#include <private/qnamedschemacomponent_p.h>
#include <private/qxsdannotated_p.h>

#include <QtCore/QList>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a XSD attribute object.
     *
     * This class represents the <em>attribute</em> object of a XML schema as described
     * <a href="http://www.w3.org/TR/xmlschema11-1/#cAttribute_Declarations">here</a>.
     *
     * It contains information from either a top-level attribute declaration (as child of
     * a <em>schema</em> object) or of a local attribute declaration (as child of <em>complexType</em>
     * or <em>attributeGroup</em> object without a 'ref' attribute).
     *
     * All other occurrences of the <em>attribute</em> object are represented by the XsdAttributeUse class.
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSAttributeDeclaration">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdAttribute : public NamedSchemaComponent, public XsdAnnotated
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdAttribute> Ptr;
            typedef QList<XsdAttribute::Ptr> List;

            /**
             * @short Describes the scope of an attribute.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#sc_a">Scope Definition</a>
             */
            class Scope : public QSharedData
            {
                public:
                    typedef QExplicitlySharedDataPointer<Scope> Ptr;

                    /**
                     * Describes the <a href="http://www.w3.org/TR/xmlschema11-1/#ad-scope">scope</a> of an attribute.
                     */
                    enum Variety
                    {
                        Global,    ///< The attribute is defined globally as child of the <em>schema</em> object.
                        Local      ///< The attribute is defined locally as child of a complex type or attribute group definition.
                    };

                    /**
                     * Sets the @p variety of the attribute scope.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#sc_a-variety">Variety Definition</a>
                     */
                    void setVariety(Variety variety);

                    /**
                     * Returns the variety of the attribute scope.
                     */
                    Variety variety() const;

                    /**
                     * Sets the @p parent complex type or attribute group definition of the attribute scope.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#sc_a-parent">Parent Definition</a>
                     */
                    void setParent(const NamedSchemaComponent::Ptr &parent);

                    /**
                     * Returns the parent complex type or attribute group definition of the attribute scope.
                     */
                    NamedSchemaComponent::Ptr parent() const;

                private:
                    Variety                   m_variety;
                    NamedSchemaComponent::Ptr m_parent;
            };


            /**
             * @short Describes the value constraint of an attribute.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#vc_a">Value Constraint Definition</a>
             */
            class ValueConstraint : public QSharedData
            {
                public:
                    typedef QExplicitlySharedDataPointer<ValueConstraint> Ptr;

                    /**
                     * Describes the <a href="http://www.w3.org/TR/xmlschema11-1/#ad-value_constraint">value constraint</a> of an attribute.
                     */
                    enum Variety
                    {
                        Default,  ///< The attribute has a default value set.
                        Fixed     ///< The attribute has a fixed value set.
                    };

                    /**
                     * Sets the @p variety of the attribute value constraint.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#vc_a-variety">Variety Definition</a>
                     */
                    void setVariety(Variety variety);

                    /**
                     * Returns the variety of the attribute value constraint.
                     */
                    Variety variety() const;

                    /**
                     * Sets the @p value of the constraint.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#vc_a-value">Value Definition</a>
                     */
                    void setValue(const QString &value);

                    /**
                     * Returns the value of the constraint.
                     */
                    QString value() const;

                    /**
                     * Sets the lexical @p form of the constraint.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#vc_a-lexical_form">Lexical Form Definition</a>
                     */
                    void setLexicalForm(const QString &form);

                    /**
                     * Returns the lexical form of the constraint.
                     */
                    QString lexicalForm() const;

                private:
                    Variety m_variety;
                    QString m_value;
                    QString m_lexicalForm;
            };

            /**
             * Sets the simple @p type definition of the attribute.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ad-type_definition">Simple Type Definition</a>
             */
            void setType(const AnySimpleType::Ptr &type);

            /**
             * Returns the simple type definition of the attribute.
             */
            AnySimpleType::Ptr type() const;

            /**
             * Sets the @p scope of the attribute.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ad-scope">Scope Definition</a>
             */
            void setScope(const Scope::Ptr &scope);

            /**
             * Returns the scope of the attribute.
             */
            Scope::Ptr scope() const;

            /**
             * Sets the value @p constraint of the attribute.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#ad-value_constraint">Value Constraint Definition</a>
             */
            void setValueConstraint(const ValueConstraint::Ptr &constraint);

            /**
             * Returns the value constraint of the attribute.
             */
            ValueConstraint::Ptr valueConstraint() const;

        private:
            AnySimpleType::Ptr   m_type;
            Scope::Ptr           m_scope;
            ValueConstraint::Ptr m_valueConstraint;
    };
}

QT_END_NAMESPACE

#endif
