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

#ifndef Patternist_AtomicType_H
#define Patternist_AtomicType_H

#include <private/qanysimpletype_p.h>
#include <private/qatomiccasterlocator_p.h>
#include <private/qatomiccomparatorlocator_p.h>
#include <private/qatomicmathematicianlocator_p.h>
#include <private/qatomictypedispatch_p.h>
#include <private/qitemtype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class Item;
    class SourceLocationReflection;

    /**
     * @short Base class for all classes that implements atomic types.
     *
     * AtomicType does not implement @c xs:anyAtomicType, it is the C++
     * base class for classes that implement atomic types, such as @c xs:anyAtomicType.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AtomicType : public ItemType,
                       public AnySimpleType
    {
    public:

        typedef QExplicitlySharedDataPointer<AtomicType> Ptr;

        virtual ~AtomicType();

        /**
         * Implements a generic algorithm which relies on wxsTypeMatches().
         *
         * @returns @c true depending on if @p item is an atomic type, and that
         * AtomicValue::itemType()'s SequenceType::itemType() matches this type.
         */
        virtual bool itemMatches(const Item &item) const;

        /**
         * @returns the result of SharedQXmlName::displayName(), of the SharedQName
         * object returned from the name() function.
         */
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * returns always @c false
         */
        virtual bool isNodeType() const;

        /**
         * returns always @c true
         */
        virtual bool isAtomicType() const;

        /**
         * Determines whether @p other is equal to this type, or is a
         * sub-type of this type.
         *
         * The implementation is generic, relying on operator==()
         * and xdtSuperType().
         */
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const;

        /**
         * @returns always 'this'
         */
        virtual ItemType::Ptr atomizedType() const;

        /**
         * @returns always SchemaType::SimpleTypeAtomic
         */
        virtual TypeCategory category() const;

        /**
         * @returns DerivationRestriction
         */
        virtual DerivationMethod derivationMethod() const;

        virtual AtomicTypeVisitorResult::Ptr
        accept(const QExplicitlySharedDataPointer<AtomicTypeVisitor> &visitor,
               const SourceLocationReflection *const) const = 0;

        virtual AtomicTypeVisitorResult::Ptr
        accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
               const qint16 param,
               const SourceLocationReflection *const) const = 0;

        virtual AtomicComparatorLocator::Ptr comparatorLocator() const = 0;
        virtual AtomicMathematicianLocator::Ptr mathematicianLocator() const = 0;
        virtual AtomicCasterLocator::Ptr casterLocator() const = 0;

    protected:
        AtomicType();

    };
}

QT_END_NAMESPACE

#endif
