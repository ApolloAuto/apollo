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

#ifndef Patternist_XsdIdcHelper_H
#define Patternist_XsdIdcHelper_H

#include <private/qreportcontext_p.h>
#include <private/qschematype_p.h>

#include <QtXmlPatterns/QXmlItem>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short A helper class for validating identity constraints.
     *
     * This class represents a field node from the key-sequence as defined in
     * the validation rules at http://www.w3.org/TR/xmlschema11-1/#d0e32243.
     */
    class FieldNode
    {
        public:
            /**
             * Creates an empty field node.
             */
            FieldNode();

            /**
             * Creates a field node that is bound to a xml node.
             *
             * @param item The xml node the field is bound to.
             * @param data The string content of that field.
             * @param type The type that is bound to that field.
             */
            FieldNode(const QXmlItem &item, const QString &data, const SchemaType::Ptr &type);

            /**
             * Returns whether this field is empty.
             *
             * A field can be empty, if the xpath expression selects an absent attribute
             * or element.
             */
            bool isEmpty() const;

            /**
             * Returns whether this field is equal to the @p other field.
             *
             * Equal means that both have the same type and there content is equal in the
             * types value space.
             */
            bool isEqualTo(const FieldNode &other, const NamePool::Ptr &namePool, const ReportContext::Ptr &context, const SourceLocationReflection *const reflection) const;

            /**
             * Returns the xml node item the field is bound to.
             */
            QXmlItem item() const;

        private:
            QXmlItem m_item;
            QString m_data;
            SchemaType::Ptr m_type;
    };

    /**
     * @short A helper class for validating identity constraints.
     *
     * This class represents a target or qualified node from the target or qualified
     * node set as defined in the validation rules at http://www.w3.org/TR/xmlschema11-1/#d0e32243.
     *
     * A target node is part of the qualified node set, if all of its fields are not empty.
     */
    class TargetNode
    {
        public:
            /**
             * Defines a set of target nodes.
             */
            typedef QSet<TargetNode> Set;

            /**
             * Creates a new target node that is bound to the xml node @p item.
             */
            explicit TargetNode(const QXmlItem &item);

            /**
             * Returns the xml node item the target node is bound to.
             */
            QXmlItem item() const;

            /**
             * Returns all xml node items, the fields of that target node are bound to.
             */
            QVector<QXmlItem> fieldItems() const;

            /**
             * Returns the number of fields that are empty.
             */
            int emptyFieldsCount() const;

            /**
             * Returns whether the target node has the same fields as the @p other target node.
             */
            bool fieldsAreEqual(const TargetNode &other, const NamePool::Ptr &namePool, const ReportContext::Ptr &context, const SourceLocationReflection *const reflection) const;

            /**
             * Adds a new field to the target node with the given values.
             */
            void addField(const QXmlItem &item, const QString &data, const SchemaType::Ptr &type);

            /**
             * Returns whether the target node is equal to the @p other target node.
             */
            bool operator==(const TargetNode &other) const;

        private:
            QXmlItem m_item;
            QVector<FieldNode> m_fields;
    };

    /**
     * Creates a hash value for the given target @p node.
     */
    inline uint qHash(const QPatternist::TargetNode &node)
    {
        return qHash(node.item().toNodeModelIndex());
    }
}

QT_END_NAMESPACE

#endif
