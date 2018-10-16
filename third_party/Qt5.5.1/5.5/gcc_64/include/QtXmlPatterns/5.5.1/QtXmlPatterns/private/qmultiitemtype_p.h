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

#ifndef Patternist_MultiItemType_H
#define Patternist_MultiItemType_H

#include <QList>

#include <private/qitemtype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents multiple types such as <tt>document()</tt> @em or <tt>xs:integer</tt>.
     *
     * In some situations two or more different types are allowed. For example, XQuery's
     * @c validate expression accepts document or element nodes(but not attribute
     * nodes, for example). MultiItemType is useful in such situations, its constructor
     * takes a list of ItemType instances which its member functions treats as a wholeness.
     *
     * For example, xdtTypeMatches() returns @c true if any of the represented types matches.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class MultiItemType : public ItemType
    {
    public:
        /**
         * Creates a MultiItemType representing the types in @p typeList. @p typeList must
         * contain two or more types.
         */
        MultiItemType(const ItemType::List &typeList);

        /**
         * The display name are the names concatenated with "|" as separator. For example,
         * if this MultiItemType represents the types <tt>document()</tt>, <tt>xs:integer</tt>,
         * and <tt>xs:anyAtomicType</tt>, the display name is
         * "document() | xs:integer | xs:anyAtomicType".
         */
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * If any of the types this MultiItemType represents matches @p item, it is
         * considered a match.
         *
         * @returns @c true if any of the housed ItemType instances matches @p item, otherwise @c false
         */
        virtual bool itemMatches(const Item &item) const;

        /**
         * If any of the types this MultiItemType represents matches @p other, it is
         * considered a match.
         *
         * @returns @c true if any of the housed ItemType instances matches @p other, otherwise @c false
         */
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const;

        /**
         * @returns @c true if any of the represented types is a node type.
         */
        virtual bool isNodeType() const;

        /**
         * @returns @c true if any of the represented types is an atomic type.
         */
        virtual bool isAtomicType() const;

        /**
         * Determines the union type of all the represented types super types. For example,
         * if the represented types are <tt>xs:integer</tt>, <tt>document()</tt>
         * and <tt>xs:string</tt>, <tt>item()</tt> is returned.
         */
        virtual ItemType::Ptr xdtSuperType() const;

        /**
         * Determines the union type of all the represented types atomized types. For example,
         * if the represented types are <tt>xs:integer</tt> and <tt>document()</tt>,
         * <tt>xs:anyAtomicType</tt> is returned, because that's the super type of <tt>xs:integer</tt>
         * and <tt>xs:untypedAtomic</tt>.
         */
        virtual ItemType::Ptr atomizedType() const;

    private:
        const ItemType::List m_types;
        const ItemType::List::const_iterator m_end;
    };
}

QT_END_NAMESPACE

#endif
