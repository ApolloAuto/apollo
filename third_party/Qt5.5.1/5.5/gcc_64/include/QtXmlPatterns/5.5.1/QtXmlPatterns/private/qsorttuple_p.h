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

#ifndef Patternist_SortTuple_H
#define Patternist_SortTuple_H

#include <private/qitem_p.h>
#include <private/qitem_p.h>
#include <private/qitemtype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a value and its sort keys
     * in FLOWR's <tt>order by</tt> clause.
     *
     * SortTuple doesn't correspond to anything in the XPath Data Model and
     * can therefore well be described as an exotic implementation detail.
     * Most of its functions asserts because it makes no sense to
     * call them.
     *
     * SortTuple exclusively exists for use with the expressions OrderBy and
     * ReturnOrderBy, and acts as a carrier between those two for sort keys and
     * source values.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class SortTuple : public AtomicValue
    {
    public:
        /**
         * @p aSortKeys may be empty.
         */
        inline SortTuple(const Item::Iterator::Ptr &aValue,
                         const Item::Vector &aSortKeys) : m_sortKeys(aSortKeys),
                                                          m_value(aValue)
        {
            Q_ASSERT(m_value);
            Q_ASSERT(!m_sortKeys.isEmpty());
        }

        /**
         * A smart pointer wrapping SortTuple instances.
         */
        typedef QExplicitlySharedDataPointer<SortTuple> Ptr;

        /**
         * This function is sometimes called by Literal::description().
         * This function simply returns "SortTuple".
         */
        virtual QString stringValue() const;

        /**
         * @short Always asserts.
         */
        virtual Item::Iterator::Ptr typedValue() const;

        /**
         * @short Always asserts.
         */
        virtual bool isAtomicValue() const;

        /**
         * @short Always asserts.
         */
        virtual bool isNode() const;

        /**
         * @short Always asserts.
         */
        virtual bool hasError() const;

        virtual ItemType::Ptr type() const;

        inline const Item::Vector &sortKeys() const
        {
            return m_sortKeys;
        }

        inline const Item::Iterator::Ptr &value() const
        {
            return m_value;
        }

    private:
        const Item::Vector          m_sortKeys;
        const Item::Iterator::Ptr   m_value;
    };
}

QT_END_NAMESPACE

#endif
