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

#ifndef Patternist_XsdSchemaParserContext_H
#define Patternist_XsdSchemaParserContext_H

#include <private/qmaintainingreader_p.h> // for definition of ElementDescription
#include <private/qxsdschematoken_p.h>
#include <private/qxsdschema_p.h>
#include <private/qxsdschemachecker_p.h>
#include <private/qxsdschemacontext_p.h>
#include <private/qxsdschemaresolver_p.h>

#include <QtCore/QSharedData>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A namespace class that contains identifiers for the different
     *        scopes a tag from the xml schema spec can appear in.
     */
    class XsdTagScope
    {
        public:
            enum Type
            {
                Schema,
                Include,
                Import,
                Redefine,
                Annotation,
                AppInfo,
                Documentation,
                GlobalSimpleType,
                LocalSimpleType,
                SimpleRestriction,
                List,
                Union,
                MinExclusiveFacet,
                MinInclusiveFacet,
                MaxExclusiveFacet,
                MaxInclusiveFacet,
                TotalDigitsFacet,
                FractionDigitsFacet,
                LengthFacet,
                MinLengthFacet,
                MaxLengthFacet,
                EnumerationFacet,
                WhiteSpaceFacet,
                PatternFacet,
                GlobalComplexType,
                LocalComplexType,
                SimpleContent,
                SimpleContentRestriction,
                SimpleContentExtension,
                ComplexContent,
                ComplexContentRestriction,
                ComplexContentExtension,
                NamedGroup,
                ReferredGroup,
                All,
                LocalAll,
                Choice,
                LocalChoice,
                Sequence,
                LocalSequence,
                GlobalAttribute,
                LocalAttribute,
                NamedAttributeGroup,
                ReferredAttributeGroup,
                GlobalElement,
                LocalElement,
                Unique,
                Key,
                KeyRef,
                Selector,
                Field,
                Notation,
                Any,
                AnyAttribute,
                Alternative,
                Assert,
                Assertion,
                OpenContent,
                DefaultOpenContent,
                Override
            };
    };

    /**
     * A hash that keeps the mapping between the single components that can appear
     * in a schema document (e.g. elements, attributes, type definitions) and their
     * source locations inside the document.
     */
    typedef QHash<NamedSchemaComponent::Ptr, QSourceLocation> ComponentLocationHash;

    /**
     * @short A context for schema parsing.
     *
     * This class provides a context for all components that are
     * nedded for parsing and compiling the XML schema.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchemaParserContext : public QSharedData
    {
        public:
            /**
             * A smart pointer wrapping XsdSchemaParserContext instances.
             */
            typedef QExplicitlySharedDataPointer<XsdSchemaParserContext> Ptr;

            /**
             * Creates a new schema parser context object.
             *
             * @param namePool The name pool where all names of the schema will be stored in.
             * @param context The schema context to use for error reporting etc.
             */
            XsdSchemaParserContext(const NamePool::Ptr &namePool, const XsdSchemaContext::Ptr &context);

            /**
             * Returns the name pool of the schema parser context.
             */
            NamePool::Ptr namePool() const;

            /**
             * Returns the schema resolver of the schema context.
             */
            XsdSchemaResolver::Ptr resolver() const;

            /**
             * Returns the schema resolver of the schema context.
             */
            XsdSchemaChecker::Ptr checker() const;

            /**
             * Returns the schema object of the schema context.
             */
            XsdSchema::Ptr schema() const;

            /**
             * Returns the element descriptions for the schema parser.
             *
             * The element descriptions are a fast lookup table for
             * verifying whether certain attributes are allowed for
             * a given element type.
             */
            ElementDescription<XsdSchemaToken, XsdTagScope::Type>::Hash elementDescriptions() const;

            /**
             * Returns an unique name that is used by the schema parser
             * for anonymous types.
             *
             * @param targetNamespace The namespace of the name.
             */
            QXmlName createAnonymousName(const QString &targetNamespace) const;

        private:
            /**
             * Fills the element description hash with the required and prohibited
             * attributes.
             */
            static ElementDescription<XsdSchemaToken, XsdTagScope::Type>::Hash setupElementDescriptions();

            NamePool::Ptr                                                     m_namePool;
            XsdSchema::Ptr                                                    m_schema;
            XsdSchemaChecker::Ptr                                             m_checker;
            XsdSchemaResolver::Ptr                                            m_resolver;
            const ElementDescription<XsdSchemaToken, XsdTagScope::Type>::Hash m_elementDescriptions;
            mutable QAtomicInt                                                m_anonymousNameCounter;
    };
}

QT_END_NAMESPACE

#endif
