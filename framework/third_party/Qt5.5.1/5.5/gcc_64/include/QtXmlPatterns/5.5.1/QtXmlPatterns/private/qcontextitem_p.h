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

#ifndef Patternist_ContextItem_H
#define Patternist_ContextItem_H

#include <private/qemptycontainer_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Implements the context item, the dot: <tt>.</tt>.
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-context-item-expression">XML Path Language
     * (XPath) 2.0, 3.1.4 Context Item Expression</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class ContextItem : public EmptyContainer
    {
    public:
        /**
         * @p expr is possibly used for error reporting. If this context item has been
         * created implicitly, such as for the expression <tt>fn:string()</tt>, @p expr
         * should be passed a valid pointer to the Expression that this context
         * item is generated for.
         */
        inline ContextItem(const Expression::Ptr &expr = Expression::Ptr()) : m_expr(expr)
        {
        }

        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
        virtual SequenceType::Ptr staticType() const;

        /**
         * @returns always DisableElimination and RequiresContextItem
         */
        virtual Expression::Properties properties() const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        /**
         * Overridden to store a pointer to StaticContext::contextItemType().
         */
        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

        /**
         * Overridden to store a pointer to StaticContext::contextItemType().
         */
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        /**
         * @returns always IDContextItem
         */
        virtual ID id() const;

        /**
         * @returns always BuiltinTypes::item;
         */
        virtual ItemType::Ptr expectedContextItemType() const;

        virtual const SourceLocationReflection *actualReflection() const;
        virtual void announceFocusType(const ItemType::Ptr &type);

    private:
        ItemType::Ptr           m_itemType;
        const Expression::Ptr   m_expr;
    };
}

QT_END_NAMESPACE

#endif
