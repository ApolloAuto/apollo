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

#ifndef Patternist_NamespaceSupport_H
#define Patternist_NamespaceSupport_H

#include <private/qnamepool_p.h>

#include <QtCore/QExplicitlySharedDataPointer>
#include <QtCore/QHash>
#include <QtCore/QSet>
#include <QtCore/QStack>
#include <QtCore/QXmlStreamNamespaceDeclarations>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A helper class for handling nested namespace declarations.
     *
     * This class is mostly an adaption of QXmlNamespaceSupport to the NamePool
     * mechanism used in XmlPatterns.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class NamespaceSupport
    {
        public:
            /**
             * Describes whether the name to process is an attribute or element.
             */
            enum NameType
            {
                AttributeName,  ///< An attribute name to process.
                ElementName     ///< An element name to process.
            };

            /**
             * Creates an empty namespace support object.
             */
            NamespaceSupport();

            /**
             * Creates a new namespace support object.
             *
             * @param namePool The name pool where all processed names are stored in.
             */
            NamespaceSupport(const NamePool::Ptr &namePool);

            /**
             * Adds a new prefix-to-namespace binding.
             *
             * @param prefixCode The name pool code for the prefix.
             * @param namespaceCode The name pool code for the namespace.
             */
            void setPrefix(const QXmlName::PrefixCode prefixCode, const QXmlName::NamespaceCode namespaceCode);

            /**
             * Adds the prefix-to-namespace bindings from @p declarations to
             * the namespace support.
             */
            void setPrefixes(const QXmlStreamNamespaceDeclarations &declarations);

            /**
             * Sets the name pool code of the target namespace of the schema the
             * namespace support works on.
             */
            void setTargetNamespace(const QXmlName::NamespaceCode code);

            /**
             * Returns the prefix code for the given namespace @p code.
             */
            QXmlName::PrefixCode prefix(const QXmlName::NamespaceCode code) const;

            /**
             * Returns the namespace code for the given prefix @p code.
             */
            QXmlName::NamespaceCode uri(const QXmlName::PrefixCode code) const;

            /**
             * Converts the given @p qualifiedName to a resolved QXmlName @p name according
             * to the current namespace mapping.
             *
             * @param qualifiedName The full qualified name.
             * @param type The type of name processing.
             * @param name The resolved QXmlName.
             *
             * @returns @c true if the name could be processed correctly or @c false if the
             *          namespace prefix is unknown.
             */
            bool processName(const QString &qualifiedName, NameType type, QXmlName &name) const;

            /**
             * Pushes the current namespace mapping onto the stack.
             */
            void pushContext();

            /**
             * Pops the current namespace mapping from the stack.
             */
            void popContext();

            /**
             * Returns the list of namespace bindings.
             */
            QList<QXmlName> namespaceBindings() const;

        private:
            typedef QHash<QXmlName::PrefixCode, QXmlName::NamespaceCode> NamespaceHash;

            NamePool::Ptr         m_namePool;
            QStack<NamespaceHash> m_nsStack;
            NamespaceHash         m_ns;
    };
}

QT_END_NAMESPACE

#endif
