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

#ifndef Patternist_AnySimpleType_H
#define Patternist_AnySimpleType_H

#include <private/qanytype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class AtomicType;

    /**
     * @short Represents the @c xs:anySimpleType item type.
     *
     * @ingroup Patternist_types
     * @see <a href="http://www.w3.org/TR/xmlschema-2/#dt-anySimpleType">XML Schema Part 2:
     * Datatypes Second Edition, The simple ur-type definition</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class AnySimpleType : public AnyType
    {
    public:
        typedef QExplicitlySharedDataPointer<AnySimpleType> Ptr;
        typedef QList<AnySimpleType::Ptr> List;
        friend class BuiltinTypes;

        virtual ~AnySimpleType();

        virtual QXmlName name(const NamePool::Ptr &np) const;

        /**
         * @returns always @c xs:anySimpleType
         */
        virtual QString displayName(const NamePool::Ptr &np) const;

        /**
         * @returns always BuiltinTypes::xsAnyType
         */
        virtual SchemaType::Ptr wxsSuperType() const;

        /**
         * xs:anySimpleType is the special "simple ur-type". Therefore this function
         * returns SchemaType::None
         *
         * @returns SchemaType::None
         */
        virtual TypeCategory category() const;

        /**
         * The simple ur-type is a "special restriction of the ur-type definition",
         * according to XML Schema Part 2: Datatypes Second Edition about xs:anySimpleType
         *
         * @returns DERIVATION_RESTRICTION
         */
        virtual SchemaType::DerivationMethod derivationMethod() const;

        /**
         * Always returns @c true.
         */
        virtual bool isSimpleType() const;

        /**
         * Always returns @c false.
         */
        virtual bool isComplexType() const;

    protected:
        AnySimpleType();

    };
}

QT_END_NAMESPACE

#endif
