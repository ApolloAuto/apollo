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

#ifndef Patternist_NumericType_H
#define Patternist_NumericType_H

#include <private/qatomictype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents the internal and abstract type @c fs:numeric.
     *
     * @see <a href="http://www.w3.org/TR/xquery-semantics/#dt-fs_numeric">XQuery 1.0
     * and XPath 2.0 Formal Semantics, Definition: fs:numeric</a>
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class NumericType : public AtomicType
    {
    public:
        virtual ~NumericType();

        virtual bool itemMatches(const Item &item) const;
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const;

        /**
         * @returns always "numeric". That is, no namespace prefix
         */
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * @returns always @c true
         */
        virtual bool isAbstract() const;

        /**
         * @returns always @c false
         */
        virtual bool isNodeType() const;

        /**
         * @returns always @c true
         */
        virtual bool isAtomicType() const;

        /**
         * @returns always xs:anyAtomicType
         */
        virtual SchemaType::Ptr wxsSuperType() const;

        /**
         * @returns always xs:anyAtomicType
         */
        virtual ItemType::Ptr xdtSuperType() const;

        /**
         * @returns @c null. It makes no sense to atomize the abstract type @c fs:numeric.
         */
        virtual ItemType::Ptr atomizedType() const;

        /**
         * NumericType cannot be visited. This function is only implemented
         * to satisfy the abstract super class's interface.
         *
         * @returns always a @c null pointer
         */
        virtual AtomicTypeVisitorResult::Ptr accept(const AtomicTypeVisitor::Ptr &visitor,
                                                    const SourceLocationReflection *const) const;

        /**
         * NumericType cannot be visited. This function is only implemented
         * to satisfy the abstract super class's interface.
         *
         * @returns always a @c null pointer
         */
        virtual AtomicTypeVisitorResult::Ptr accept(const ParameterizedAtomicTypeVisitor::Ptr &visitor,
                                                    const qint16 op,
                                                    const SourceLocationReflection *const) const;

        /**
         * The type @c fs:numeric is an abstract type which therefore
         * cannot be involved in comparisons. Hence, this function returns
         * @c null. This function is only implemented to satisfy the abstract
         * super class's interface.
         *
         * @returns always a @c null pointer
         */
        virtual AtomicComparatorLocator::Ptr comparatorLocator() const;

        /**
         * The type @c fs:numeric is an abstract type which therefore
         * cannot be involved in arithmetics. Hence, this function returns
         * @c null. This function is only implemented to satisfy the abstract
         * super class's interface.
         *
         * @returns always a @c null pointer
         */
        virtual AtomicMathematicianLocator::Ptr mathematicianLocator() const;


        /**
         * The type @c fs:numeric is an abstract type which therefore
         * cannot be involved in casting. Hence, this function returns
         * @c null. This function is only implemented to satisfy the abstract
         * super class's interface.
         *
         * @returns always a @c null pointer
         */
        virtual AtomicCasterLocator::Ptr casterLocator() const;

    protected:
        friend class BuiltinTypes;
        NumericType();
    };
}

QT_END_NAMESPACE

#endif
