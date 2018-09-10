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

#ifndef Patternist_XsdWildcard_H
#define Patternist_XsdWildcard_H

#include <private/qxsdterm_p.h>

#include <QtCore/QSet>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a XSD wildcard object.
     *
     * This class represents the <em>wildcard</em> object of a XML schema as described
     * <a href="http://www.w3.org/TR/xmlschema11-1/#Wildcards">here</a>.
     *
     * It contains information from either an <em>any</em> object or an <em>anyAttribute</em> object.
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSWildcard">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdWildcard : public XsdTerm
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdWildcard> Ptr;

            /**
             * Defines the absent namespace that is used in wildcards.
             */
            static QString absentNamespace();

            /**
             * Describes the <a href="http://www.w3.org/TR/xmlschema11-1/#w-namespace_constraint">namespace constraint</a> of the wildcard.
             */
            class NamespaceConstraint : public QSharedData
            {
                public:
                    typedef QExplicitlySharedDataPointer<NamespaceConstraint> Ptr;

                    /**
                     * Describes the variety of the namespace constraint.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#nc-variety">Variety Definition</a>
                     */
                    enum Variety
                    {
                        Any,         ///< Any namespace is allowed.
                        Enumeration, ///< Namespaces in the namespaces set are allowed.
                        Not          ///< Namespaces in the namespaces set are not allowed.
                    };

                    /**
                     * Sets the @p variety of the namespace constraint.
                     *
                     * @see <a href="http://www.w3.org/TR/xmlschema11-1/#nc-variety">Variety Definition</a>
                     */
                    void setVariety(Variety variety);

                    /**
                     * Returns the variety of the namespace constraint.
                     */
                    Variety variety() const;

                    /**
                     * Sets the set of @p namespaces of the namespace constraint.
                     */
                    void setNamespaces(const QSet<QString> &namespaces);

                    /**
                     * Returns the set of namespaces of the namespace constraint.
                     */
                    QSet<QString> namespaces() const;

                    /**
                     * Sets the set of disallowed @p names of the namespace constraint.
                     */
                    void setDisallowedNames(const QSet<QString> &names);

                    /**
                     * Returns the set of disallowed names of the namespace constraint.
                     */
                    QSet<QString> disallowedNames() const;

                private:
                    Variety       m_variety;
                    QSet<QString> m_namespaces;
                    QSet<QString> m_disallowedNames;
            };

            /**
             * Describes the <a href="http://www.w3.org/TR/xmlschema11-1/#w-process_contents">type of content processing</a> of the wildcard.
             */
            enum ProcessContents
            {
                Strict,      ///< There must be a top-level declaration for the item available, or the item must have an xsi:type, and the item must be valid as appropriate.
                Lax,         ///< If the item has a uniquely determined declaration available, it must be valid with respect to that definition.
                Skip         ///< No constraints at all: the item must simply be well-formed XML.
            };

            /**
             * Creates a new wildcard object.
             */
            XsdWildcard();

            /**
             * Returns always @c true, used to avoid dynamic casts.
             */
            virtual bool isWildcard() const;

            /**
             * Sets the namespace @p constraint of the wildcard.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#w-namespace_constraint">Namespace Constraint Definition</a>
             */
            void setNamespaceConstraint(const NamespaceConstraint::Ptr &constraint);

            /**
             * Returns the namespace constraint of the wildcard.
             */
            NamespaceConstraint::Ptr namespaceConstraint() const;

            /**
             * Sets the process @p contents of the wildcard.
             *
             * @see <a href="http://www.w3.org/TR/xmlschema11-1/#w-process_contents">Process Contents Definition</a>
             */
            void setProcessContents(ProcessContents contents);

            /**
             * Returns the process contents of the wildcard.
             */
            ProcessContents processContents() const;

        private:
            NamespaceConstraint::Ptr m_namespaceConstraint;
            ProcessContents          m_processContents;
    };
}

QT_END_NAMESPACE

#endif
