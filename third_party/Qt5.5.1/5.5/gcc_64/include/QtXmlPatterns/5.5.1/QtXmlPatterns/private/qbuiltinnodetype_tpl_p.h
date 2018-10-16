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
//

/**
 * @file
 * @short This file is included by BuiltintNodeType.h.
 * If you need includes in this file, put them in BuiltintNodeType.h, outside of the namespace.
 */

template <const QXmlNodeModelIndex::NodeKind kind>
BuiltinNodeType<kind>::BuiltinNodeType()
{
}

template <const QXmlNodeModelIndex::NodeKind kind>
bool BuiltinNodeType<kind>::xdtTypeMatches(const ItemType::Ptr &other) const
{
    if(!other->isNodeType())
        return false;

    return *static_cast<const BuiltinNodeType *>(other.data()) == *this
            ? true
            : xdtTypeMatches(other->xdtSuperType());
}

template <const QXmlNodeModelIndex::NodeKind kind>
bool BuiltinNodeType<kind>::itemMatches(const Item &item) const
{
    Q_ASSERT(item);

    return item.isNode() &&
           item.asNode().kind() == kind;
}

template <const QXmlNodeModelIndex::NodeKind kind>
ItemType::Ptr BuiltinNodeType<kind>::atomizedType() const
{
    switch(kind)
    {
        /* Fallthrough all these. */
        case QXmlNodeModelIndex::Attribute:
        case QXmlNodeModelIndex::Document:
        case QXmlNodeModelIndex::Element:
        case QXmlNodeModelIndex::Text:
            return BuiltinTypes::xsUntypedAtomic;
        case QXmlNodeModelIndex::ProcessingInstruction:
        /* Fallthrough. */
        case QXmlNodeModelIndex::Comment:
            return BuiltinTypes::xsString;
        default:
        {
            Q_ASSERT_X(false, Q_FUNC_INFO,
                       "Encountered invalid XPath Data Model node type.");
            return BuiltinTypes::xsUntypedAtomic;
        }
    }
}

template <const QXmlNodeModelIndex::NodeKind kind>
QString BuiltinNodeType<kind>::displayName(const NamePool::Ptr &) const
{
    switch(kind)
    {
        case QXmlNodeModelIndex::Element:
            return QLatin1String("element()");
        case QXmlNodeModelIndex::Document:
            return QLatin1String("document()");
        case QXmlNodeModelIndex::Attribute:
            return QLatin1String("attribute()");
        case QXmlNodeModelIndex::Text:
            return QLatin1String("text()");
        case QXmlNodeModelIndex::ProcessingInstruction:
            return QLatin1String("processing-instruction()");
        case QXmlNodeModelIndex::Comment:
            return QLatin1String("comment()");
        default:
        {
            Q_ASSERT_X(false, Q_FUNC_INFO,
                       "Encountered invalid XPath Data Model node type.");
            return QString();
        }
    }
}

template <const QXmlNodeModelIndex::NodeKind kind>
ItemType::Ptr BuiltinNodeType<kind>::xdtSuperType() const
{
    return BuiltinTypes::node;
}

template <const QXmlNodeModelIndex::NodeKind kind>
QXmlNodeModelIndex::NodeKind BuiltinNodeType<kind>::nodeKind() const
{
    return kind;
}

template <const QXmlNodeModelIndex::NodeKind kind>
PatternPriority BuiltinNodeType<kind>::patternPriority() const
{
    /* See XSL Transformations (XSLT) Version 2.0, 6.4 Conflict Resolution for
     * Template Rules */
    switch(kind)
    {
        case QXmlNodeModelIndex::Text:
        /* Fallthrough. */
        case QXmlNodeModelIndex::ProcessingInstruction:
        /* Fallthrough. */
        case QXmlNodeModelIndex::Comment:
            /* "If the pattern is any other NodeTest, optionally preceded by a
             * PatternAxis, then the priority is 0.5."
             * Fallthrough. */
        case QXmlNodeModelIndex::Attribute:
        /* Fallthrough. */
        case QXmlNodeModelIndex::Element:
        /* Fallthrough. */
        case QXmlNodeModelIndex::Document:
            /* "If the pattern has the form /, then the priority is -0.5.". */
            return -0.5;
        default:
        {
            Q_ASSERT_X(false, Q_FUNC_INFO, "Unknown node type");
            return 0;
        }
    }

}

