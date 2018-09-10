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

#ifndef Patternist_NoneType_H
#define Patternist_NoneType_H

#include <private/qatomictype_p.h>
#include <private/qsequencetype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Represents the special <tt>none</tt> type.
     *
     * @ingroup Patternist_types
     * @see <a href="http://www.w3.org/TR/xquery-semantics/#sec_content_models">XQuery 1.0 and
     * XPath 2.0 Formal Semantics, 2.4.3 Content models</a>
     * @see <a href="http://www.w3.org/TR/xquery-semantics/#sec_fnerror">XQuery 1.0 and XPath 2.0
     * Formal Semantics, 7.2.9 The fn:error function</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class NoneType : public ItemType,
                     public SequenceType
    {
    public:
        typedef QExplicitlySharedDataPointer<NoneType> Ptr;

        virtual bool itemMatches(const Item &item) const;
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const;

        /**
         * @returns always "none". That is, no namespace prefix
         */
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * @note The semantical meaning of this type's item type can
         * surely be discussed. The function is provided due to
         * it being mandated by the SequenceType base class.
         *
         * @returns always 'this' since NoneType is also an ItemType
         */
        virtual ItemType::Ptr itemType() const;

        /**
         * @note The semantical meaning of this type's cardinality
         * can surely be discussed. The function is provided due to
         * it being mandated by the SequenceType base class.
         *
         * @returns always Cardinality::zeroOrMore()
         */
        virtual Cardinality cardinality() const;

        /**
         * @returns always @c false
         */
        virtual bool isAtomicType() const;

        /**
         * This can be thought to be a weird function for this type(none). There
         * is no atomized type for none, perhaps the best from a conceptual perspective
         * would be to return @c null.
         *
         * This function returns BuiltinTypes::xsAnyAtomicType because
         * the generic type checking code inserts an Atomizer in the AST
         * when an error() function(or other node which has type none) is part of
         * an operator expression(value/general comparison, arithmetics). The Atomizer
         * returns the atomizedType() of its child, and by here returning xsAnyAtomicType,
         * static operator lookup is postponed to runtime. Subsequently, expressions like error()
         * works properly with other XPath expressions.
         */
        virtual ItemType::Ptr atomizedType() const;

        /**
         * @returns always @c false
         */
        virtual bool isNodeType() const;

        /**
         * @returns always item()
         */
        virtual ItemType::Ptr xdtSuperType() const;

        /**
         * @returns always @p other. The none type can be thought as
         * disappearing when attempting to find the union of it and
         * another type.
         */
        virtual const ItemType &operator|(const ItemType &other) const;

    protected:

        friend class CommonSequenceTypes;
        NoneType();
    };
}

QT_END_NAMESPACE

#endif
