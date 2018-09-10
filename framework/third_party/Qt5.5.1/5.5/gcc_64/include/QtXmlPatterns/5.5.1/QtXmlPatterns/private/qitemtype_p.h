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

#ifndef Patternist_ItemType_H
#define Patternist_ItemType_H

#include <QSharedData>

#include <QtXmlPatterns/private/qnamepool_p.h>

QT_BEGIN_NAMESPACE

template<typename T> class QList;

namespace QPatternist
{
    class Item;

    /**
     * @short Base class for the XPath Data Model's type hierarchy.
     *
     * It can not be instantiated, but it's possible via ItemType's two subtypes:
     * Nodes, represented by QXmlNodeModelIndex, and atom types, represented by AtomicType.
     *
     * ItemType tries to by its design stay close to the notation used in Formal Semantics.
     * The operator|() is a good example, it allow typing code to be written
     * similar to how inference rules in the specification are written.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ItemType : public virtual QSharedData
    {
    public:
        /**
         * A smart pointer wrapping ItemType instances.
         */
        typedef QExplicitlySharedDataPointer<ItemType> Ptr;
        /**
         * A list of ItemType instances, each wrapped in a smart pointer.
         */
        typedef QList<ItemType::Ptr> List;

        virtual ~ItemType();

        enum Category
        {
            NodeNameTest = 1,
            Other        = 2
        };

        /**
         * Determines whether this ItemType is equal to @p other.
         *
         * Many types are represented by singleton instances. For example, there
         * exists only one instance of IntegerType. This operator==() takes advantage
         * of that and uses equalness of object addresses for determining semantic
         * equalness. This function is as a result fast.
         *
         * However, it's overridden in some cases, such as for name tests, where
         * it's not guaranteed that there exists two types.
         *
         * @returns @c true if this ItemType is equal to @p other, otherwise @c false.
         */
        virtual bool operator==(const ItemType &other) const;

        /**
         * @returns the result of operator==() negated.
         */
        inline bool operator!=(const ItemType &other) const;

        /**
         * @returns a string representing the type. Used for diagnostic purposes. For a
         * type whose name is a QName, a lexical representation should be returned
         * with the prefix being a conventional one. Examples of a display names
         * are "item()" and "xs:nonPositiveInteger".
         */
        virtual QString displayName(const NamePool::Ptr &np) const = 0;

        /**
         * @param item the item that is to be matched. This is guaranteed by the caller
         * to never be @c null.
         */
        virtual bool itemMatches(const Item &item) const = 0;

        /**
         * @short Returns @c true if @p other matches this type. That is, if @p
         * other is equal to this type or a subtype of this type.
         *
         * For instance this statements evaluates to @c true:
         *
         * @code
         * BuiltinTypes::xsAnyAtomicType->xdtTypeMatches(BuiltinTypes::xsString);
         * @endcode
         *
         * but this evaluates to @c false:
         *
         * @code
         * BuiltinTypes::attribute->xdtTypeMatches(BuiltinTypes::node);
         * @endcode
         *
         * @param other the other ItemType that is to be matched. This is guaranteed by the caller
         * to never be @c null.
         */
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const = 0;

        virtual bool isNodeType() const = 0;
        virtual bool isAtomicType() const = 0;

        /**
         * Determines the type's parent type in the XPath Data Model hierarchy. For example,
         * for the type xs:anyAtomicType, the super type in the XPath Data Model is item(), not
         * xs:anySimpleType. SchemaType::xdtSuperType navigates the schema hierarchy.
         *
         * @see SchemaType::wxsSuperType()
         * @returns the type's super type.
         */
        virtual ItemType::Ptr xdtSuperType() const = 0;

        /**
         * @todo docs mention union, give if-expression example.
         *
         * Determines the super type that is closest to this ItemType and @p other. That is,
         * the parent type of them both. For example, for the type xs:integer and xs:string
         * the parent type is xs:anyAtomicType. For xs:NOTATION and processing-instruction(), it
         * is item(), to name another example.
         *
         * This function can be seen as the type function prime(Type), defined in Formal Semantics.
         *
         * This walks the XPath Data Model type hierarchy, not the W3C XML Schema hierarchy.
         * @param other the item type 'this' object, should be compared with. Invoking xdtSuperType
         * on 'this' object with @p other as argument yields the same result as invoking the
         * function on @p other with 'this'
         * as argument.
         * @returns the parent type of 'this' and @p other
         * @see <a href="http://www.w3.org/TR/xquery-semantics/\#jd_prime">XQuery 1.0 and XPath 2.0
         * Formal Semantics, Prime Types, type function prime(Type)</a>
         */
        virtual const ItemType &operator|(const ItemType &other) const;

        /**
         * Determines the atomic type that the resulting sequence after
         * atomization of this node would be an instance of. For example, for document node,
         * xs:untypedAtomic is returned. Phrased differently, the returned type is the
         * type of the result of the typed-value accessor.
         *
         * If the type cannot be atomized, it returns @c null.
         *
         * This function is also defined on SchemaType, because some schema types can also be
         * atomized.
         *
         * @see SchemaType::atomizedType()
         * @see <a href="http://www.w3.org/TR/xpath-datamodel/\#dm-typed-value">XQuery 1.0
         * and XPath 2.0 Data Model, 5.15 typed-value Accessor</a>
         * @see <a href="http://www.w3.org/TR/xquery-semantics/#jd_data">XQuery 1.0
         * and XPath 2.0 Formal Semantics, data on auxiliary judgment</a>
         * @returns the atomic type that the resulting sequence
         * when performing atomization is an instance of.
         */
        virtual ItemType::Ptr atomizedType() const = 0;

        /**
         * @returns always Other
         */
        virtual Category itemTypeCategory() const;

        enum InstanceOf
        {
            ClassLocalNameTest,
            ClassNamespaceNameTest,
            ClassQNameTest,
            ClassOther
        };

        /**
         * Determines what class this ItemType is an instance of. This
         * is in needed in some implementations of operator operator==(). By
         * default, Other is returned.
         */
        virtual InstanceOf instanceOf() const;

        inline ItemType()
        {
        }

    private:
        Q_DISABLE_COPY(ItemType)
    };

    /**
     * This operator exists for making it easier to use the ItemType class, which
     * always are wrapped in ItemType::Ptr, by taking care of the dereferencing
     * of ItemType::Ptr instances. Semantically, it performs the same as
     * ItemType's operator of the same name.
     *
     * @relates ItemType
     * @see ItemType::operator|()
     * @see operator|=(ItemType::Ptr &, const ItemType::Ptr &)
     */
    inline ItemType::Ptr operator|(const ItemType::Ptr &op1,
                                   const ItemType::Ptr &op2)
    {
        return ItemType::Ptr(const_cast<ItemType *>(&(*op1 | *op2)));
    }

    bool ItemType::operator!=(const ItemType &other) const
    {
        return this != &other;
    }

    /**
     * @short Computes the union type of @p op1 and @p op2, and assigns it to @p op1.
     *
     * This operator exists for making it easier to use the ItemType class, which
     * always are wrapped in ItemType::Ptr, by taking care of the dereferencing
     * of the ItemType::Ptr instances.
     *
     * @relates ItemType
     * @see operator|(const ItemType::Ptr &, const ItemType::Ptr &)
     * @param op1 if @c null, @p op2 is returned unchanged
     * @param op2 the other operand
     */
    inline void operator|=(ItemType::Ptr &op1, const ItemType::Ptr &op2)
    {
        op1 = op1 | op2;
    }

}

Q_DECLARE_TYPEINFO(QPatternist::ItemType::Ptr, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
