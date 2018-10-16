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

#ifndef Patternist_BuiltinTypes_H
#define Patternist_BuiltinTypes_H

#include <private/qanynodetype_p.h>
#include <private/qanysimpletype_p.h>
#include <private/qanytype_p.h>
#include <private/qbuiltinatomictype_p.h>
#include <private/qitemtype_p.h>
#include <private/qnumerictype_p.h>
#include <private/quntyped_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Provides access to singleton instances of ItemType and SchemaType sub-classes.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT BuiltinTypes
    {
    public:
        static const SchemaType::Ptr        xsAnyType;
        static const SchemaType::Ptr        xsAnySimpleType;
        static const SchemaType::Ptr        xsUntyped;

        static const AtomicType::Ptr        xsAnyAtomicType;
        static const AtomicType::Ptr        xsUntypedAtomic;
        static const AtomicType::Ptr        xsDateTime;
        static const AtomicType::Ptr        xsDate;
        static const AtomicType::Ptr        xsTime;
        static const AtomicType::Ptr        xsDuration;
        static const AtomicType::Ptr        xsYearMonthDuration;
        static const AtomicType::Ptr        xsDayTimeDuration;

        /**
         * An artificial type for implementation purposes
         * that represents the XPath type @c numeric.
         */
        static const AtomicType::Ptr        numeric;
        static const AtomicType::Ptr        xsFloat;
        static const AtomicType::Ptr        xsDouble;
        static const AtomicType::Ptr        xsInteger;
        static const AtomicType::Ptr        xsDecimal;
        static const AtomicType::Ptr        xsNonPositiveInteger;
        static const AtomicType::Ptr        xsNegativeInteger;
        static const AtomicType::Ptr        xsLong;
        static const AtomicType::Ptr        xsInt;
        static const AtomicType::Ptr        xsShort;
        static const AtomicType::Ptr        xsByte;
        static const AtomicType::Ptr        xsNonNegativeInteger;
        static const AtomicType::Ptr        xsUnsignedLong;
        static const AtomicType::Ptr        xsUnsignedInt;
        static const AtomicType::Ptr        xsUnsignedShort;
        static const AtomicType::Ptr        xsUnsignedByte;
        static const AtomicType::Ptr        xsPositiveInteger;


        static const AtomicType::Ptr        xsGYearMonth;
        static const AtomicType::Ptr        xsGYear;
        static const AtomicType::Ptr        xsGMonthDay;
        static const AtomicType::Ptr        xsGDay;
        static const AtomicType::Ptr        xsGMonth;

        static const AtomicType::Ptr        xsBoolean;

        static const AtomicType::Ptr        xsBase64Binary;
        static const AtomicType::Ptr        xsHexBinary;
        static const AtomicType::Ptr        xsAnyURI;
        static const AtomicType::Ptr        xsQName;
        static const AtomicType::Ptr        xsString;
        static const AtomicType::Ptr        xsNormalizedString;
        static const AtomicType::Ptr        xsToken;
        static const AtomicType::Ptr        xsLanguage;
        static const AtomicType::Ptr        xsNMTOKEN;
        static const AtomicType::Ptr        xsName;
        static const AtomicType::Ptr        xsNCName;
        static const AtomicType::Ptr        xsID;
        static const AtomicType::Ptr        xsIDREF;
        static const AtomicType::Ptr        xsENTITY;

        static const AtomicType::Ptr        xsNOTATION;
        static const ItemType::Ptr          item;

        static const AnyNodeType::Ptr       node;

        /**
         * When the node test node() is used without axes in a pattern in
         * XSL-T, it doesn't match document nodes. See 5.5.3 The Meaning of a
         * Pattern.
         *
         * This node test does that.
         */
        static const ItemType::Ptr          xsltNodeTest;

        static const ItemType::Ptr          attribute;
        static const ItemType::Ptr          comment;
        static const ItemType::Ptr          document;
        static const ItemType::Ptr          element;
        static const ItemType::Ptr          pi;
        static const ItemType::Ptr          text;

    private:
        /**
         * The constructor is protected because this class is not meant to be instantiated,
         * but should only be used via its static const members.
         */
        BuiltinTypes();
        Q_DISABLE_COPY(BuiltinTypes)
    };
}

QT_END_NAMESPACE

#endif

