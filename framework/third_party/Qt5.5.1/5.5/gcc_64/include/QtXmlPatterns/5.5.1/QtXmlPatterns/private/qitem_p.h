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

#ifndef Patternist_Item_H
#define Patternist_Item_H

#include <QtXmlPatterns/private/qcppcastinghelper_p.h>
#include <QtXmlPatterns/private/qitemtype_p.h>
#include <QtXmlPatterns/private/qsingletoniterator_p.h>
#include <QtXmlPatterns/QAbstractXmlNodeModel>

#include <QUrl>
#include <QVariant>

/**
 * @file
 * @short Due to strong interdependencies, this file contains the definitions for
 * the classes Item, QXmlNodeModelIndex, QAbstractXmlNodeModel and AtomicValue. The implementations are
 * in their respective source files.
 */

/**
 * @class QSharedData
 * @short Qt's base class for reference counting.
 */

QT_BEGIN_NAMESPACE

template<typename T> class QList;
template<typename T> class QVector;
template<typename T> class QAbstractXmlForwardIterator;

class QSourceLocation;
class QAbstractXmlReceiver;

namespace QPatternist
{
    class DynamicContext;
    class Item;
    class ItemType;
    class QObjectNodeModel;
    template<typename T> class EmptyIterator;
    template<typename T, typename ListType> class ListIterator;

    /**
     * @short Base class for all classes representing atomic values.
     *
     * Instantiating AtomicValues sub classes from a value of somekind,
     * for a certain type is done in three different ways:
     *
     * - The static factory fromLexical which available in most classes. This
     * function attempts to create a value from a QString that is considered
     * a lexical representation of the value. Thus, this function performs validation, takes
     * care of whitespace facets, and everything else related to instantiating a value from
     * a lexical representation.
     * - The static factory function fromValue. This function exists for
     * values where a C++ type exists which corresponds to the type's value space.
     * - By using instances available in CommonValues. This is the preferred method
     * since it uses existing singleton instances and thus saves memory. CommonValues
     * should be used whenever possible, it can be thought as a collection of constant values.
     *
     * For types that does not distinguish the value space and lexical space, such as <tt>xs:string</tt>,
     * only the fromValue() function exist, and fromLexical() is omitted.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AtomicValue : public QSharedData
                      , public CppCastingHelper<AtomicValue>
    {
    public:
        virtual ~AtomicValue();

        /**
         * A smart pointer wrapping AtomicValue instances.
         */
        typedef QExplicitlySharedDataPointer<AtomicValue> Ptr;

        /**
         * A list if smart pointers wrapping AtomicValue instances.
         */
        typedef QList<AtomicValue::Ptr> List;

        /**
         * Determines whether this atomic value has an error. This is used
         * for implementing casting.
         *
         * @returns always @c false
         */
        virtual bool hasError() const;

        /**
         * Always fails by issuing the type error ReportContext::FORG0006. Sub-classes
         * whose represented type do allow EBV to be extracted from, must thus
         * re-implement this function.
         */
        virtual bool evaluateEBV(const QExplicitlySharedDataPointer<DynamicContext> &context) const;

        virtual QString stringValue() const = 0;
        virtual ItemType::Ptr type() const = 0;

        /**
         * Converts @p value to a QVariant.
         */
        static QVariant toQt(const AtomicValue *const value);

        static inline QVariant toQt(const AtomicValue::Ptr &value)
        {
            return toQt(value.data());
        }

        static Item toXDM(const QVariant &value);

        static ItemType::Ptr qtToXDMType(const QXmlItem &item);
    protected:
        inline AtomicValue()
        {
        }
    };

    /**
     * @short Represents an item in the XPath 2.0 Data Model.
     *
     * There exists two types of items: nodes and atomic values.
     *
     * The XQuery 1.0 and XPath 2.0 Data Model and XML Path Language (XPath) 2.0 specification
     * makes a very strong distinction between a sequence of items and an atomized sequence.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Item
    {
        friend class QT_PREPEND_NAMESPACE(QXmlItem);

    public:
        /**
         * A smart pointer wrapping an Item instance.
         */
        typedef QAbstractXmlForwardIterator<Item> Iterator;

        /**
         * A list of Item instances, each wrapped in a smart pointer.
         */
        typedef QList<Item> List;

        /**
         * A vector of Item instances, each wrapped in a smart pointer.
         */
        typedef QVector<Item> Vector;

        typedef QPatternist::SingletonIterator<Item> SingletonIterator;
        typedef QPatternist::EmptyIterator<Item> EmptyIterator;

        /**
         * Default constructor.
         */
        inline Item()
        {
            reset();
        }

        inline Item(const QXmlNodeModelIndex &n) : node(n.m_storage)
        {
        }

        inline Item(const Item &other) : node(other.node)
        {
            Q_ASSERT_X(sizeof(QXmlNodeModelIndex) >= sizeof(AtomicValue), Q_FUNC_INFO,
                       "Since we're only copying the node member, it must be the largest.");
            if(isAtomicValue())
                atomicValue->ref.ref();
        }

        inline Item(const AtomicValue::Ptr &a)
        {
            reset();
            if(a)
            {
                atomicValue = a.data();
                atomicValue->ref.ref();

                /* Signal that we're housing an atomic value. */
                node.model = reinterpret_cast<const QAbstractXmlNodeModel *>(~0);
            }
        }

        inline Item(const AtomicValue *const a)
        {
            /* Note, the implementation is a copy of the constructor above. */
            reset();
            if(a)
            {
                atomicValue = a;
                atomicValue->ref.ref();

                /* Signal that we're housing an atomic value. */
                node.model = reinterpret_cast<const QAbstractXmlNodeModel *>(~0);
            }
        }

        inline ~Item()
        {
            if(isAtomicValue() && !atomicValue->ref.deref())
                delete atomicValue;
        }

        inline Item &operator=(const Item &other)
        {
            Q_ASSERT_X(sizeof(QXmlNodeModelIndex) >= sizeof(AtomicValue *), Q_FUNC_INFO,
                       "If this doesn't hold, we won't copy all data.");

            if(other.isAtomicValue())
                other.atomicValue->ref.ref();

            if(isAtomicValue())
            {
                if(!atomicValue->ref.deref())
                    delete atomicValue;
            }

            node = other.node;

            return *this;
        }

        template<typename TCastTarget>
        inline TCastTarget *as() const
        {
#if defined(Patternist_DEBUG) && !defined(Q_CC_XLC)
/* At least on aix-xlc-64, the compiler cries when it sees dynamic_cast. */
            Q_ASSERT_X(atomicValue == 0 || dynamic_cast<const TCastTarget *>(atomicValue),
                       Q_FUNC_INFO,
                       "The cast is invalid. This class does not inherit the cast target.");
#endif
            return const_cast<TCastTarget *>(static_cast<const TCastTarget *>(atomicValue));
        }

        /**
         * @short Returns the string value of this Item.
         *
         * In the case of a node, it is the node value corresponding to
         * the particular node type. For atomic values, it is equivalent
         * to the value cast as <tt>xs:string</tt>.
         *
         * Conceptually, this functions corresponds to the <tt>dm:string-value</tt> accessor.
         *
         * @see <a href="http://www.w3.org/TR/xpath-datamodel/#dm-string-value">XQuery 1.0 and
         * XPath 2.0 Data Model, 5.13 string-value Accessor</a>
         * @returns the string value.
         */
        inline QString stringValue() const
        {
            if(isAtomicValue())
                return atomicValue->stringValue();
            else
                return asNode().stringValue();
        }

        /**
         * @short Returns the typed value of this item.
         *
         * Conceptually, this functions corresponds to the <tt>dm:typed-value</tt> accessor. Here are
         * examples of what the typed value of an Item is:
         *
         * - The typed value of an atomic value is always the atomic value itself.
         * - A comment node has always a typed value of type @c xs:string
         * - For attribute and element nodes, the typed value can be arbitrary. For example, an
         *   element can have a sequence of @c xs:dateTime instances.
         *
         * @returns the typed value of this item
         * @see <a href="http://www.w3.org/TR/xpath-datamodel/#dm-typed-value">XQuery 1.0 and
         * XPath 2.0 Data Model, 5.15 typed-value Accessor</a>
         */
        Item::Iterator::Ptr sequencedTypedValue() const;

        /**
         * @short Determines whether this item is an atomic value, or a node.
         *
         * If this Item is @c null, @c false is returned.
         *
         * @see isNode()
         * @returns @c true if it is an atomic value, otherwise @c false.
         */
        inline bool isAtomicValue() const
        {
            /* Setting node.model to ~0, signals that it's an atomic value. */
            return node.model == reinterpret_cast<QAbstractXmlNodeModel *>(~0);
        }

        /**
         * @short Determines whether this item is an atomic value, or a node.
         *
         * If this Item is @c null, false is returned.
         *
         * @see isAtomicValue()
         * @returns @c true if this item is a node, otherwise @c false.
         */
        inline bool isNode() const
        {
            //return !isAtomicValue();
            return node.model && node.model != reinterpret_cast<QAbstractXmlNodeModel *>(~0);
        }

        /**
         * @short Returns the ItemType this Item is of.
         *
         * For example, if this Item is an XML node, more specifically a text node,
         * <tt>text()</tt> is returned. That is, BuiltinTypes::text. However, if this
         * Item is an atomic value of type <tt>xs:long</tt> that is what's returned,
         * BuiltinTypes::xsLong.
         *
         * @returns the type of this Item.
         */
        inline QExplicitlySharedDataPointer<ItemType> type() const
        {
            if(isAtomicValue())
                return atomicValue->type();
            else
                return asNode().type();
        }

        inline const AtomicValue *asAtomicValue() const
        {
            Q_ASSERT(isAtomicValue());
            return atomicValue;
        }

        inline const QXmlNodeModelIndex &asNode() const
        {
            Q_ASSERT_X(isNode() || isNull(), Q_FUNC_INFO,
                       "This item isn't a valid QXmlNodeModelIndex.");
            Q_ASSERT_X(sizeof(QXmlNodeModelIndex) == sizeof(QPatternist::NodeIndexStorage), Q_FUNC_INFO,
                       "If this doesn't hold, something is wrong.");

            return reinterpret_cast<const QXmlNodeModelIndex &>(node);
        }

        inline operator bool() const
        {
            return node.model;
        }

        inline bool isNull() const
        {
            return !node.model;
        }

        inline void reset()
        {
            node.reset();
        }

        static inline Item fromPublic(const QXmlItem &i)
        {
            const Item it(i.m_node);
            if(it.isAtomicValue())
                it.asAtomicValue()->ref.ref();

            return it;
        }

        static inline QXmlItem toPublic(const Item &i)
        {
            return QXmlItem(i);
        }

    private:
        union
        {
            NodeIndexStorage node;
            const AtomicValue *atomicValue;
        };
    };

    template<typename T>
    inline Item toItem(const QExplicitlySharedDataPointer<T> atomicValue)
    {
        return Item(atomicValue.data());
    }

    /**
     * This is an overload, provided for convenience.
     * @relates QXmlNodeModelIndex
     */
    static inline QString formatData(const QXmlNodeModelIndex node)
    {
        return node.stringValue(); // This can be improved a lot.
    }
}

    inline QXmlName QXmlNodeModelIndex::name() const
    {
        return m_storage.model->name(*this);
    }

    inline QXmlNodeModelIndex QXmlNodeModelIndex::root() const
    {
        return m_storage.model->root(*this);
    }

    inline QXmlNodeModelIndex::Iterator::Ptr QXmlNodeModelIndex::iterate(const QXmlNodeModelIndex::Axis axis) const
    {
        return m_storage.model->iterate(*this, axis);
    }

    inline QUrl QXmlNodeModelIndex::documentUri() const
    {
        return m_storage.model->documentUri(*this);
    }

    inline QUrl QXmlNodeModelIndex::baseUri() const
    {
        return m_storage.model->baseUri(*this);
    }

    inline QXmlNodeModelIndex::NodeKind QXmlNodeModelIndex::kind() const
    {
        return m_storage.model->kind(*this);
    }

    inline bool QXmlNodeModelIndex::isDeepEqual(const QXmlNodeModelIndex &other) const
    {
        return m_storage.model->isDeepEqual(*this, other);
    }

    inline QXmlNodeModelIndex::DocumentOrder QXmlNodeModelIndex::compareOrder(const QXmlNodeModelIndex &other) const
    {
        Q_ASSERT_X(model() == other.model(), Q_FUNC_INFO, "The API docs guarantees the two nodes are from the same model");
        return m_storage.model->compareOrder(*this, other);
    }

    inline bool QXmlNodeModelIndex::is(const QXmlNodeModelIndex &other) const
    {
        return m_storage.model == other.m_storage.model &&
               m_storage.data == other.m_storage.data &&
               m_storage.additionalData == other.m_storage.additionalData;
    }

    inline void QXmlNodeModelIndex::sendNamespaces(QAbstractXmlReceiver *const receiver) const
    {
        m_storage.model->sendNamespaces(*this, receiver);
    }

    inline QVector<QXmlName> QXmlNodeModelIndex::namespaceBindings() const
    {
        return m_storage.model->namespaceBindings(*this);
    }

    inline QXmlName::NamespaceCode QXmlNodeModelIndex::namespaceForPrefix(const QXmlName::PrefixCode prefix) const
    {
        return m_storage.model->namespaceForPrefix(*this, prefix);
    }

    inline QString QXmlNodeModelIndex::stringValue() const
    {
        return m_storage.model->stringValue(*this);
    }

    inline QPatternist::ItemType::Ptr QXmlNodeModelIndex::type() const
    {
        return m_storage.model->type(*this);
    }

    inline QExplicitlySharedDataPointer<QAbstractXmlForwardIterator<QPatternist::Item> > QXmlNodeModelIndex::sequencedTypedValue() const
    {
        return m_storage.model->sequencedTypedValue(*this);
    }

    inline QXmlItem::QXmlItem(const QPatternist::Item &i) : m_node(i.node)
    {
        if(isAtomicValue())
            m_atomicValue->ref.ref();
    }

Q_DECLARE_TYPEINFO(QPatternist::Item::Iterator::Ptr, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(QPatternist::AtomicValue, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
