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

#ifndef Patternist_SchemaType_H
#define Patternist_SchemaType_H

#include <private/qnamepool_p.h>
#include <private/qschemacomponent_p.h>
#include <QXmlName>

template<typename N, typename M> class QHash;
template<typename N> class QList;

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class AtomicType;

    /**
     * @short Base class for W3C XML Schema types.
     *
     * This is the base class of all data types in a W3C XML Schema.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class SchemaType : public SchemaComponent
    {
    public:

        typedef QExplicitlySharedDataPointer<SchemaType> Ptr;
        typedef QHash<QXmlName, SchemaType::Ptr> Hash;
        typedef QList<SchemaType::Ptr> List;

        /**
         * Schema types are divided into different categories such as
         * complex type, atomic imple type, union simple type, and so forth. This
         * enumerator, which category() returns a value of, identifies what
         * category the type belong to.
         *
         * @todo Add docs & links for the enums
         */
        enum TypeCategory
        {
            None = 0,
            /**
             * A simple atomic type. These are also sometimes
             * referred to as primitive types. An example of this type is
             * xs:string.
             *
             * Formally speaking, a simple type with variety atomic.
             */
            SimpleTypeAtomic,
            SimpleTypeList,
            SimpleTypeUnion,
            ComplexType
        };

        enum DerivationMethod
        {
            DerivationRestriction   = 1,
            DerivationExtension     = 2,
            DerivationUnion         = 4,
            DerivationList          = 8,
            /**
             * Used for <tt>xs:anyType</tt>.
             */
            NoDerivation            = 16
        };

        /**
         * Describes the derivation constraints that are given by the 'final' or 'block' attributes.
         */
        enum DerivationConstraint
        {
            RestrictionConstraint = 1,
            ExtensionConstraint = 2,
            ListConstraint = 4,
            UnionConstraint = 8
        };
        Q_DECLARE_FLAGS(DerivationConstraints, DerivationConstraint)

        SchemaType();
        virtual ~SchemaType();

        /**
         * Determines how this SchemaType is derived from its super type.
         *
         * @note Despite that DerivationMethod is designed for being
         * used for bitwise OR'd value, this function may only return one enum
         * value. If the type does not derive from any type, which is the case of
         * <tt>xs:anyType</tt>, this function returns NoDerivation.
         *
         * @see SchemaType::wxsSuperType()
         * @see <a href="http://www.w3.org/TR/DOM-Level-3-Core/core.html#TypeInfo-DerivationMethods">Document
         * Object Model (DOM) Level 3 Core Specification, Definition group DerivationMethods</a>
         * @returns a DerivationMethod enumerator signifiying how
         * this SchemaType is derived from its base type
         */
        virtual DerivationMethod derivationMethod() const = 0;

        /**
         * Determines what derivation constraints exists for the type.
         */
        virtual DerivationConstraints derivationConstraints() const = 0;

        /**
         * Determines whether the type is an abstract type.
         *
         * @note It is important a correct value is returned, since
         * abstract types must not be instantiated.
         */
        virtual bool isAbstract() const = 0;

        /**
         * @short Returns the name of the type.
         *
         * The reason to why we take the name pool argument, is that the basic
         * types, @c xs:anySimpleType and so on, are stored globally in
         * BuiltinTypes and ComonSequenceTypes, and therefore cannot be tied to
         * a certain name pool. Type instances that knows they always will be
         * used with a certain name pool, can therefore ignore @p np and return
         * a QXmlName instance stored as a member.
         *
         * If the type code was refactored to not be store globally and
         * therefore by design would be tied to a name pool, this argument could
         * be removed.
         */
        virtual QXmlName name(const NamePool::Ptr &np) const = 0;

        /**
         * @short Returns a suitable display name for this type.
         *
         * See name() for an explanation to why we take a NamePool as argument.
         */
        virtual QString displayName(const NamePool::Ptr &np) const = 0;

        /**
         * @returns the W3C XML Schema base type that this type derives from. All types
         * returns an instance, except for the xs:anyType since it
         * is the very base type of all types, and it returns 0. Hence,
         * one can walk the hierarchy of a schema type by recursively calling
         * wxsSuperType, until zero is returned.
         *
         * This function walks the Schema hierarchy. Some simple types, the atomic types,
         * is also part of the XPath Data Model hierarchy, and their super type in that
         * hierarchy can be introspected with xdtSuperType().
         *
         * wxsSuperType() can be said to correspond to the {base type definition} property
         * in the Post Schema Valid Infoset(PSVI).
         *
         * @see ItemType::xdtSuperType()
         */
        virtual SchemaType::Ptr wxsSuperType() const = 0;

        /**
         * @returns @c true if @p other is identical to 'this' schema type or if @p other
         * is either directly or indirectly a base type of 'this'. Hence, calling
         * AnyType::wxsTypeMatches() with @p other as argument returns @c true for all types,
         * since all types have @c xs:anyType as super type.
         */
        virtual bool wxsTypeMatches(const SchemaType::Ptr &other) const = 0;

        virtual TypeCategory category() const = 0;

        /**
         * Determines whether the type is a simple type, by introspecting
         * the result of category().
         *
         * @note Do not re-implement this function, but instead override category()
         * and let it return an appropriate value.
         */
        virtual bool isSimpleType() const;

        /**
         * Determines whether the type is a complex type, by introspecting
         * the result of category().
         *
         * @note Do not re-implement this function, but instead override category()
         * and let it return an appropriate value.
         */
        virtual bool isComplexType() const;

        /**
         * Returns whether the value has been defined by a schema (is not a built in type).
         */
        virtual bool isDefinedBySchema() const;
    };

    Q_DECLARE_OPERATORS_FOR_FLAGS(SchemaType::DerivationConstraints)
}

QT_END_NAMESPACE

#endif
