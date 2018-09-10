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

#ifndef Patternist_XsdInstanceReader_H
#define Patternist_XsdInstanceReader_H

#include "qabstractxmlnodemodel.h"
#include <private/qpullbridge_p.h>
#include <private/qxsdschemacontext_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short The schema instance reader.
     *
     * This class reads in a xml instance document from a QAbstractXmlNodeModel
     * and provides a QXmlStreamReader like interface with some additional context
     * information.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdInstanceReader
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdInstanceReader> Ptr;

            /**
             * Creates a new instance reader that will read the data from
             * the given @p model.
             *
             * @param model The model the data are read from.
             * @param context The context that is used for error reporting etc.
             */
            XsdInstanceReader(const QAbstractXmlNodeModel *model, const XsdSchemaContext::Ptr &context);

        protected:
            /**
             * Returns @c true if the end of the document is reached, @c false otherwise.
             */
            bool atEnd() const;

            /**
             * Reads the next node from the document.
             */
            void readNext();

            /**
             * Returns whether the current node is a start element.
             */
            bool isStartElement() const;

            /**
             * Returns whether the current node is an end element.
             */
            bool isEndElement() const;

            /**
             * Returns whether the current node has a text node among its children.
             */
            bool hasChildText() const;

            /**
             * Returns whether the current node has an element node among its children.
             */
            bool hasChildElement() const;

            /**
             * Returns the name of the current node.
             */
            QXmlName name() const;

            /**
             * Returns whether the current node has an attribute with the given @p name.
             */
            bool hasAttribute(const QXmlName &name) const;

            /**
             * Returns the attribute with the given @p name of the current node.
             */
            QString attribute(const QXmlName &name) const;

            /**
             * Returns the list of attribute names of the current node.
             */
            QSet<QXmlName> attributeNames() const;

            /**
             * Returns the concatenated text of all direct child text nodes.
             */
            QString text() const;

            /**
             * Converts a qualified name into a QXmlName according to the namespace
             * mappings of the current node.
             */
            QXmlName convertToQName(const QString &name) const;

            /**
             * Returns a source location object for the current position.
             */
            QSourceLocation sourceLocation() const;

            /**
             * Returns the QXmlItem for the current position.
             */
            QXmlItem item() const;

            /**
             * Returns the QXmlItem for the attribute with the given @p name at the current position.
             */
            QXmlItem attributeItem(const QXmlName &name) const;

            /**
             * Returns the namespace bindings for the given node model @p index.
             */
            QVector<QXmlName> namespaceBindings(const QXmlNodeModelIndex &index) const;

            /**
             * The shared schema context.
             */
            XsdSchemaContext::Ptr     m_context;

        private:
            PullBridge                m_model;
            QHash<QXmlName, QString>  m_cachedAttributes;
            QHash<QXmlName, QXmlItem> m_cachedAttributeItems;
            QSourceLocation           m_cachedSourceLocation;
            QXmlItem                  m_cachedItem;
    };
}

QT_END_NAMESPACE

#endif
